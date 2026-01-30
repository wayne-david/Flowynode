#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "lora.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "OLEDDisplay.h"
#include "driver/ledc.h"
#include "protocol.h"
#include <inttypes.h>


#define tag "FLOWYNODE"
#define NODEID "N1"
#define FW_VERSION "1.0.0"

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)
#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */

// GPIOs for DONE and HELP buttons and rotary encoder
#define DONE_BUTTON_GPIO 32
#define HELP_BUTTON_GPIO 33
#define ROTARY_CLK_GPIO 25
#define ROTARY_DT_GPIO 26
#define ROTARY_SW_GPIO 27

// Buzzer output GPIO
#define BUZZER_GPIO 15
#define BUZZER_PWM_FREQ  3000 // 3kHz, typical for passive buzzers
#define BUZZER_PWM_DUTY  4000 // out of 8191 (50%)
#define BUZZER_PWM_TIMER LEDC_TIMER_0
#define BUZZER_PWM_MODE  LEDC_LOW_SPEED_MODE
#define BUZZER_PWM_CH    LEDC_CHANNEL_0

SemaphoreHandle_t print_mux = NULL;
SemaphoreHandle_t data_mutex;

typedef enum {
    STATE_IDLE = 0,
    STATE_DONE,
    STATE_DONE_OFF,
    STATE_HELP,
    STATE_HELP_OFF,
    STATE_TIME_REQ,
    STATE_TIME_APPROVED   // +X
} node_state_t;
static node_state_t current_state = STATE_IDLE;
volatile int32_t approved_minutes = 0;   // for +X
char requested_minutes[8];   // for +X

typedef enum{
  HOME,
  TIME_REQ,
} screens_t;
screens_t current_screen = HOME;

#define MSG_MAX_LEN 32
typedef struct {
    uint8_t data[MSG_MAX_LEN];
    uint16_t len;
} msg_t;
static uint32_t  msg_id = 0;

typedef struct {
    char time_global[8];
    char time_local[8];
    char battery[8];
    char time_req[8];
    char status[16]; 
} screen_data_t;
static screen_data_t screen_data;
static volatile bool screen_changed = true;

static QueueHandle_t lora_rx_queue;
static QueueHandle_t lora_tx_queue;

char msg[PROTO_MAX_MSG_LEN];

//Timer 
typedef struct {
    int32_t seconds;     // remaining time in seconds
    int32_t offset;      // time offset in seconds
    bool running;        // counting or paused
} node_timer_t;
static node_timer_t timer = {0, 0, false};
static TimerHandle_t timer_tick;

static void timer_tick_cb(TimerHandle_t xTimer)
{
    if (timer.running && timer.seconds > 0) {
        timer.seconds--;
        screen_changed = true; 
    }
    else if(timer.running && timer.offset > 0){
        timer.offset--;
        screen_changed = true; 
    }
}

//function to generate zero-padded 4 digits, wrap at 9999 → 0001
static const char *generate_msg_id(void)
{
    static char msg_id_str[5];
    msg_id++;
    if (msg_id > 9999) {
        msg_id = 1;
    }
    snprintf(msg_id_str, sizeof(msg_id_str), "%04"PRIu32, msg_id);
    return msg_id_str;
}

const char *state_to_string(node_state_t state, char *buf, size_t buf_len)
{
    switch (state) {
        case STATE_DONE:
            return "DONE";
        
        case STATE_DONE_OFF:
            return "DONE OFF";

        case STATE_HELP:
            return "HELP";

        case STATE_HELP_OFF:
            return "HELP OFF";

        case STATE_TIME_REQ: {
            snprintf(buf, buf_len, "REQ : +%s", requested_minutes);
            return buf;
        }

        case STATE_TIME_APPROVED:
            snprintf(buf, buf_len, "TIME +%" PRId32, approved_minutes);
            return buf;

        case STATE_IDLE:
        default:
            return "READY";
    }
}

static void buzzer_init_pwm(void) {
   ledc_timer_config_t timer_conf = {
      .speed_mode = BUZZER_PWM_MODE,
      .timer_num = BUZZER_PWM_TIMER,
      .duty_resolution = LEDC_TIMER_13_BIT,
      .freq_hz = BUZZER_PWM_FREQ,
      .clk_cfg = LEDC_AUTO_CLK
   };
   ledc_timer_config(&timer_conf);

   ledc_channel_config_t ch_conf = {
      .gpio_num = BUZZER_GPIO,
      .speed_mode = BUZZER_PWM_MODE,
      .channel = BUZZER_PWM_CH,
      .timer_sel = BUZZER_PWM_TIMER,
      .duty = 0,
      .hpoint = 0
   };
   ledc_channel_config(&ch_conf);
}

static void buzzer_beep(int ms) {
   // Start PWM
   ledc_set_duty(BUZZER_PWM_MODE, BUZZER_PWM_CH, BUZZER_PWM_DUTY);
   ledc_update_duty(BUZZER_PWM_MODE, BUZZER_PWM_CH);
   vTaskDelay(pdMS_TO_TICKS(ms));
   // Stop PWM
   ledc_set_duty(BUZZER_PWM_MODE, BUZZER_PWM_CH, 0);
   ledc_update_duty(BUZZER_PWM_MODE, BUZZER_PWM_CH);
}

static void gpio_init_buzzer(void) {
   gpio_config_t io_conf = {
      .intr_type = GPIO_INTR_DISABLE,
      .mode = GPIO_MODE_OUTPUT,
      .pin_bit_mask = (1ULL << BUZZER_GPIO),
      .pull_down_en = 0,
      .pull_up_en = 0
   };
   gpio_config(&io_conf);
   gpio_set_level(BUZZER_GPIO, 0);
}

static void gpio_init_inputs(void) {
   gpio_config_t io_conf = {
      .intr_type = GPIO_INTR_DISABLE,
      .mode = GPIO_MODE_INPUT,
      .pin_bit_mask = (1ULL << DONE_BUTTON_GPIO) | (1ULL << HELP_BUTTON_GPIO) |
                  (1ULL << ROTARY_CLK_GPIO) | (1ULL << ROTARY_DT_GPIO) | (1ULL << ROTARY_SW_GPIO),
      .pull_down_en = 0,
      .pull_up_en = 1
   };
   gpio_config(&io_conf);
}

//Screen update methods
void screen_set_battery(const char *battery)
{
    strncpy(screen_data.battery, battery,
            sizeof(screen_data.battery) - 1);
    screen_data.battery[sizeof(screen_data.battery) - 1] = '\0';

    screen_changed = true;
}

void screen_set_time(int time_seconds)  //in seconds
{
    //convert timer seconds to const char* for display  
    int global_minutes = (time_seconds / 60) % 100;
    int global_seconds = time_seconds % 60;

    int local_minutes = ((time_seconds + timer.offset)/ 60) % 100;
    int local_seconds = (time_seconds + timer.offset) % 60;

    snprintf(screen_data.time_global, sizeof(screen_data.time_global), "%02d:%02d", global_minutes, global_seconds);
    snprintf(screen_data.time_local, sizeof(screen_data.time_local), "%02d:%02d", local_minutes, local_seconds);

    // int minutes = (timer.seconds / 60) % 100;
    // int seconds = timer.seconds % 60;

    // snprintf(screen_data.time_global, sizeof(screen_data.time_global), "%02d:%02d", minutes, seconds);
    // snprintf(screen_data.time_local, sizeof(screen_data.time_local), "%02d:%02d", minutes, seconds);

    screen_changed = true;
}

void screen_set_time_req(int time_seconds)  //in seconds
{
    // convert timer seconds to const char* for display  
    int minutes = (time_seconds / 60) % 100;
    int seconds = time_seconds % 60;
    
    snprintf(requested_minutes, sizeof(requested_minutes), "%02d", minutes);
    snprintf(screen_data.time_req, sizeof(screen_data.time_req), "%02d:%02d", minutes, seconds);
    screen_changed = true;
}

void screen_set_state(node_state_t state)
{
    char tmp[16];
    const char *s = state_to_string(state, tmp, sizeof(tmp));

    strncpy(screen_data.status, s, sizeof(screen_data.status) - 1);
    screen_data.status[sizeof(screen_data.status) - 1] = '\0';
    screen_changed = true;
}

//Dynamic display update function for FlowyHub
void task_update_screen(void *p) {
    OLEDDisplay_t *oled = OLEDDisplay_init(I2C_MASTER_NUM,0x78,I2C_MASTER_SDA_IO,I2C_MASTER_SCL_IO);
    OLEDDisplay_flipScreenVertically(oled);
    screen_data_t screen_msg;
    char time_offset[12]; 
               
    for(;;) {
        if (!screen_changed)
        {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        screen_changed = false;
       
        if (xSemaphoreTake(data_mutex, 0) == pdTRUE) {
            int global_minutes = (timer.seconds / 60) % 100;
            int global_seconds = timer.seconds % 60;

            int local_minutes = ((timer.seconds + timer.offset)/ 60) % 100;
            int local_seconds = (timer.seconds + timer.offset) % 60;
            int offset_minutes = (timer.offset/60);

            snprintf(screen_data.time_global, sizeof(screen_data.time_global), "%02d:%02d", global_minutes, global_seconds);
            snprintf(screen_data.time_local, sizeof(screen_data.time_local), "%02d:%02d", local_minutes, local_seconds);
            snprintf(time_offset, sizeof(time_offset), "+%02d", offset_minutes);

            screen_msg = screen_data;
            xSemaphoreGive(data_mutex);
        }
        
        OLEDDisplay_clear(oled);
        // //First Row
        OLEDDisplay_setTextAlignment(oled,TEXT_ALIGN_LEFT);
        OLEDDisplay_setFont(oled,ArialMT_Plain_10);
        OLEDDisplay_drawString(oled,00, 00, screen_msg.battery);

        OLEDDisplay_setTextAlignment(oled,TEXT_ALIGN_CENTER);
        OLEDDisplay_setFont(oled,ArialMT_Plain_10);
        OLEDDisplay_drawString(oled,64, 00, screen_msg.time_global);

        OLEDDisplay_setTextAlignment(oled,TEXT_ALIGN_CENTER);
        OLEDDisplay_setFont(oled,ArialMT_Plain_10);
        OLEDDisplay_drawString(oled,120, 00, NODEID);

        switch (current_screen) {
            case HOME:
                OLEDDisplay_setTextAlignment(oled,TEXT_ALIGN_CENTER);
                OLEDDisplay_setFont(oled,ArialMT_Plain_24);
                OLEDDisplay_drawString(oled,64, 20, screen_msg.time_local);

                OLEDDisplay_setTextAlignment(oled,TEXT_ALIGN_RIGHT);
                OLEDDisplay_setFont(oled,ArialMT_Plain_10);
                OLEDDisplay_drawString(oled,120, 25, time_offset);

                // OLEDDisplay_display(oled);
                // vTaskDelay(pdMS_TO_TICKS(50));

                OLEDDisplay_setTextAlignment(oled, TEXT_ALIGN_LEFT);
                OLEDDisplay_setFont(oled, ArialMT_Plain_10);
                OLEDDisplay_drawString(oled, 00, 54, screen_msg.status);
                OLEDDisplay_display(oled);
                vTaskDelay(pdMS_TO_TICKS(50));
                break;

            case TIME_REQ:
                OLEDDisplay_setTextAlignment(oled,TEXT_ALIGN_CENTER);
                OLEDDisplay_setFont(oled,ArialMT_Plain_24);
                OLEDDisplay_drawString(oled,64, 20, screen_msg.time_req);

                OLEDDisplay_display(oled);
                vTaskDelay(pdMS_TO_TICKS(50));
                break;

            default:
                break;

        } 
       vTaskDelay(pdMS_TO_TICKS(50)); 
    } 
}

void task_input(void *p) {
    int last_done = 1, last_help = 1;
    int last_clk = 1, last_dt = 1, last_sw = 1;
    bool rotary_active = false;
    bool done_active = false;
    bool help_active = false;

    msg_t msg;
    static const int8_t rotary_table[16] = {
        0, -1, +1,  0,
        +1,  0,  0, -1,
        -1,  0,  0, +1,
        0, +1, -1,  0
    };
    uint8_t last_rotary_state = 0;
    int8_t rotary_accum = 0;
    int rotary_state = 0;

    while (1) {
        int done = gpio_get_level(DONE_BUTTON_GPIO);
        int help = gpio_get_level(HELP_BUTTON_GPIO);
        int clk = gpio_get_level(ROTARY_CLK_GPIO);
        int dt = gpio_get_level(ROTARY_DT_GPIO);
        int sw = gpio_get_level(ROTARY_SW_GPIO);

        // DONE button press
        if (last_done == 1 && done == 0) {
            if(!done_active){
                done_active = true;
                memset(&msg, 0, sizeof(msg));
                const char *msg_id = generate_msg_id();
                proto_done((char *)msg.data, sizeof(msg.data), NODEID, msg_id);
                msg.len = strlen((char *)msg.data);
                xQueueSend(lora_tx_queue, &msg, 0);
                current_state = STATE_DONE;
                screen_set_state(current_state);
            }
            else{
                done_active = false;
                memset(&msg, 0, sizeof(msg));
                const char *msg_id = generate_msg_id();
                proto_done_off((char *)msg.data, sizeof(msg.data), NODEID, msg_id);
                msg.len = strlen((char *)msg.data);
                xQueueSend(lora_tx_queue, &msg, 0);
                current_state = STATE_DONE_OFF;
                screen_set_state(current_state);
            }
            buzzer_beep(250);
            ESP_LOGI("INPUT", "Event Sent: %.*s", msg.len, msg.data);
        }
        last_done = done;

        // HELP button press
        if (last_help == 1 && help == 0) {
            if(!help_active){
                help_active = true;

                memset(&msg, 0, sizeof(msg));
                const char *msg_id = generate_msg_id();
                proto_help((char *)msg.data, sizeof(msg.data), NODEID, msg_id);
                msg.len = strlen((char *)msg.data);
                xQueueSend(lora_tx_queue, &msg, 0);
                current_state = STATE_HELP;
                screen_set_state(current_state);
            }
            else{
                help_active = false;

                memset(&msg, 0, sizeof(msg));
                const char *msg_id = generate_msg_id();
                proto_help_off((char *)msg.data, sizeof(msg.data), NODEID, msg_id);
                msg.len = strlen((char *)msg.data);
                xQueueSend(lora_tx_queue, &msg, 0);
                current_state = STATE_HELP_OFF;
                screen_set_state(current_state);
            }
            buzzer_beep(250);
            ESP_LOGI("INPUT", "Event Sent: %.*s", msg.len, msg.data);
        }
        last_help = help;

        // SW button press
        if (last_sw == 1 && sw == 0) {
            if (!rotary_active) {
                // Enable rotary and switch screen
                rotary_active = true;
                current_screen = TIME_REQ;  // switch to time request screen
                screen_changed = true;

                buzzer_beep(250);
                ESP_LOGI("INPUT", "Rotary enabled, current value: %d", rotary_state);

                // Show initial rotary value
                screen_set_time_req(rotary_state * 60);
            } else {
                // Send rotary value and go back to HOME screen
                char minutes[6];
                snprintf(minutes, sizeof(minutes), "%d", rotary_state);

                memset(&msg, 0, sizeof(msg));
                const char *msg_id = generate_msg_id();
                proto_time((char *)msg.data, sizeof(msg.data), NODEID, minutes, msg_id);
                msg.len = strlen((char *)msg.data);
                xQueueSend(lora_tx_queue, &msg, 0);

                current_state = STATE_TIME_REQ;
                screen_set_state(current_state);
                
                buzzer_beep(250);
                ESP_LOGI("INPUT", "Rotary Event Sent: %.*s", msg.len, msg.data);

                rotary_active = false;
                current_screen = HOME;  // back to home screen
                screen_changed = true;
            }
        }
        last_sw = sw;

        if (rotary_active) {
            uint8_t clk = gpio_get_level(ROTARY_CLK_GPIO);
            uint8_t dt  = gpio_get_level(ROTARY_DT_GPIO);
            uint8_t new_state = (clk << 1) | dt;

            if (new_state != last_rotary_state) {
                int8_t delta = rotary_table[(last_rotary_state << 2) | new_state];

                if (delta != 0) {
                    rotary_accum += delta;

                    // One detent = 4 transitions
                    if (rotary_accum >= 4) {
                        rotary_state++;
                        rotary_accum = 0;
                    } else if (rotary_accum <= -4) {
                        rotary_state--;
                        rotary_accum = 0;
                    }

                    // Clamp
                    if (rotary_state < 0) rotary_state = 0;
                    if (rotary_state > 99) rotary_state = 99;

                    if (current_screen == TIME_REQ) {
                        screen_set_time_req(rotary_state * 60);
                    }
                }

                last_rotary_state = new_state;
            }
        }
        last_clk = clk;
        last_dt = dt;

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void task_lora_rx_tx(void *p)
{
    int x;
    msg_t msg;
    msg_t tx_msg;
    uint8_t buf[MSG_MAX_LEN];

    lora_receive();

    for (;;){
        if (lora_received()){
            x = lora_receive_packet(buf, sizeof(buf));
            buf[x] = 0;

            //ESP_LOGI("LORA", "RX: %s | RSSI=%d", buf, lora_packet_rssi());

            memset(&msg, 0, sizeof(msg));  
            strncpy((char*)msg.data, (const char*)buf, MSG_MAX_LEN-1);
            msg.len = strlen((const char*)msg.data);

            xQueueSend(lora_rx_queue, &msg, 0);
        }

        if (xQueueReceive(lora_tx_queue, &tx_msg, 0) == pdTRUE)
        {
            //ESP_LOGI("LORA SEND", "Sending: %.*s", sizeof(tx_msg.data), tx_msg.data);
            lora_send_packet(tx_msg.data, tx_msg.len);
        }
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

void task_lora_parse(void *p)
{
    msg_t message;
    msg_t tx_msg;
    protocol_parsed_msg_t parsed;
    bool send_ack = false;
    for (;;) {
        if (xQueueReceive(lora_rx_queue, &message, portMAX_DELAY) == pdTRUE) {
            if (parse_inbound_message((char *)message.data, &parsed)) {
                
                if (parsed.node_id[0] != '\0' && strcmp(parsed.node_id, NODEID) != 0) {
                    // Not for this node, skip
                    vTaskDelay(pdMS_TO_TICKS(10));
                    continue;
                }

                switch (parsed.type) {
                    case IN_CMD_HELLO_ACK:
                        memset(&tx_msg, 0, sizeof(tx_msg));
                        const char *msg_id = generate_msg_id();
                        proto_ver((char *)tx_msg.data, sizeof(tx_msg.data), NODEID,FW_VERSION, msg_id);
                        xQueueSend(lora_tx_queue, &tx_msg, portMAX_DELAY);
                        break;

                    case IN_CMD_TSET:
                        timer.seconds = parsed.value * 60;  // minutes -> seconds
                        screen_set_time(timer.seconds);
                        timer.running = false;
                        send_ack = true;
                        break;

                    case IN_CMD_TGO:
                        if (timer.seconds > 0) timer.running = true;
                        send_ack = true;
                        // node powered off
                        break;

                    case IN_CMD_TSTOP:
                        timer.seconds = 0; 
                        screen_set_time(timer.seconds);
                        timer.running = false;    
                        break;

                    case IN_CMD_TSYNC:
                        timer.seconds = parsed.value;
                        screen_set_time(timer.seconds);
                        send_ack = true;
                        // sync time
                        break;

                    case IN_CMD_TADD:
                        timer.offset += parsed.value*60;
                        approved_minutes = parsed.value;
                        current_state = STATE_TIME_APPROVED;
                        screen_set_state(current_state);
                        send_ack = true;
                        // add time offset for node
                        break;

                    case IN_CMD_CLEAR_OFFSET:
                        timer.offset = 0;
                        timer.seconds = 0;  
                        send_ack = true;
                        // clear time offset for node
                        break;

                    case IN_CMD_CLEAR_OFFSET_ALL:
                        timer.offset = 0;
                        timer.seconds = 0;  
                        // time offset for all nodes
                        break;

                    case IN_CMD_STATUS_SET:
                        // set status flag for node
                        break;

                    case IN_CMD_STATUS_CLEAR:
                        // clear status flag for node
                        break;

                    case IN_CMD_STATUS_CLEAR_ALL:
                        // clear all status flags
                        break;

                    case IN_CMD_RESET:
                        send_ack = true;
                        // reset node
                        break;

                    case IN_CMD_RESET_ALL:
                        // reset all nodes
                        break;

                    case IN_CMD_SLEEP:
                        send_ack = true;
                        // put node to sleep
                        break;

                    case IN_CMD_WAKE:
                        // wake node from sleep
                        break;

                    case IN_CMD_ACK:
                        // received ACK
                        break;

                    case IN_CMD_CFG_BEACONS:
                        // configure beacons
                        break;
                        
                    default:
                        break;
                }

                //Send ACK
                if (send_ack && parsed.msg_id > 0) {
                    // Send ACK back after random delay  between 500 and 1000 to avoid collisions
                    vTaskDelay(pdMS_TO_TICKS(500 + (rand() % 1000)));
                   
                    msg_t tx_msg;
                    char msg_id_str[5];  
                    snprintf(msg_id_str, sizeof(msg_id_str),"%04"PRIu32, parsed.msg_id);
                    // proto_ack(msg, sizeof(msg), proto_cmd_inbound_to_str(parsed.type),NODEID,msg_id_str);
                    // xQueueSend(lora_tx_queue, &msg, 0);
                    
                    memset(&tx_msg, 0, sizeof(tx_msg));
                    proto_ack((char *)tx_msg.data, sizeof(tx_msg.data),proto_cmd_inbound_to_str(parsed.type), NODEID,msg_id_str);
                    xQueueSend(lora_tx_queue, &tx_msg, portMAX_DELAY);

                    //ESP_LOGI("ACK", "ACK Event Sent: %s", msg_id_str);

                }

            } else {
                ESP_LOGW("LORA_PARSE", "Invalid message: %s", message.data);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void task_send_ping(void *p)
{
    msg_t ping_msg;
    char msg_buf[PROTO_MAX_MSG_LEN];
    uint8_t buf[MSG_MAX_LEN];

    for (;;)
    {
        const char *msg_id = generate_msg_id();
        proto_ping(msg_buf, sizeof(msg_buf), NODEID, msg_id);

        memset(&ping_msg, 0, sizeof(ping_msg));
        strncpy((char *)ping_msg.data, msg_buf, sizeof(ping_msg.data) - 1);
        ping_msg.len = strlen((char *)ping_msg.data);

        // NON-BLOCKING send (0 ticks)
        if (xQueueSend(lora_tx_queue, &ping_msg, 0) != pdTRUE)
        {
            ESP_LOGW(tag, "Ping queue full, message dropped");
        }

        // This is fine – it yields the CPU, not blocking other tasks
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main()
{
    esp_log_level_set("*", ESP_LOG_NONE);

    while (!lora_init()) {
        ESP_LOGI("LORA", "Waiting for LoRa...");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    lora_set_sync_word(0xF3);
    lora_set_frequency(868E6);
    lora_receive();    // put into receive mode
    lora_enable_crc();

    ESP_LOGI("LORA", "LoRa initialized successfully");

    data_mutex = xSemaphoreCreateMutex();
    assert(data_mutex != NULL);

    gpio_init_inputs();
    gpio_init_buzzer();
    buzzer_init_pwm();

    lora_rx_queue = xQueueCreate(8, sizeof(msg_t));
    lora_tx_queue = xQueueCreate(8, sizeof(msg_t));
  
    timer_tick = xTimerCreate(
        "NodeTimer",
        pdMS_TO_TICKS(1000),
        pdTRUE,  // auto-reload
        NULL,
        timer_tick_cb
    );
    xTimerStart(timer_tick, 0);

    // convert timer seconds to const char* for display
    char tmp[16];
    int minutes = (0 / 60) % 100;
    int seconds = 0 % 60;

    snprintf(screen_data.time_global, sizeof(screen_data.time_global), "00:00");
    snprintf(screen_data.time_local, sizeof(screen_data.time_local), "%02d:%02d", minutes, seconds);
    snprintf(screen_data.battery, sizeof(screen_data.battery), "85%%");
    const char *s = state_to_string(current_state, tmp, sizeof(tmp));
    strncpy(screen_data.status, s, sizeof(screen_data.status) - 1);
    screen_data.status[sizeof(screen_data.status) - 1] = '\0';
    screen_changed = true;


    xTaskCreatePinnedToCore(task_lora_rx_tx, "task_lora_tx_rx", 4096, NULL, 6, NULL,1);
    xTaskCreatePinnedToCore(task_input, "task_input", 4096, NULL, 5, NULL,1);

    xTaskCreatePinnedToCore(task_update_screen, "update_screen_task", 8192, NULL, 5, NULL,0);
    xTaskCreatePinnedToCore(task_lora_parse, "task_lora_parse", 4096, NULL, 5, NULL,0);
    xTaskCreatePinnedToCore(task_send_ping, "task_send_ping", 2048, NULL, 5, NULL,0);
    
    ESP_LOGI("STACK", "free=%d",
         uxTaskGetStackHighWaterMark(NULL));
    vTaskDelay(pdMS_TO_TICKS(1000));

    const char *msg_id = generate_msg_id();
    proto_hello(msg, sizeof(msg),NODEID, msg_id);
    xQueueSend(lora_tx_queue, &msg, 0);
    ESP_LOGI("MAIN", "Event Sent: %s", msg);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


