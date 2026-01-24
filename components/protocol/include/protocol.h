#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define PROTO_MAX_MSG_LEN   128
extern const char *protocol_cmd_inbound_str[];
extern const char *protocol_cmd_outbound_str[];

typedef enum {
    IN_CMD_UNKNOWN = 0,
    IN_CMD_HELLO_ACK,
    IN_CMD_TSET,
    IN_CMD_TGO,
    IN_CMD_TSTOP,
    IN_CMD_TSYNC,
    IN_CMD_TADD,
    IN_CMD_CLEAR_OFFSET,
    IN_CMD_CLEAR_OFFSET_ALL,
    IN_CMD_STATUS_SET,
    IN_CMD_STATUS_CLEAR,
    IN_CMD_STATUS_CLEAR_ALL,
    IN_CMD_RESET,
    IN_CMD_RESET_ALL,
    IN_CMD_SLEEP,
    IN_CMD_WAKE,
    IN_CMD_ACK,
    IN_CMD_CFG_BEACONS,
    IN_CMD_MAX
} protocol_cmd_inbound_t;


extern const char *protocol_cmd_outbound_str[];
typedef enum {
    OUT_CMD_UNKNOWN = 0,
    OUT_CMD_HELLO,
    OUT_CMD_VER,
    OUT_CMD_PING,
    OUT_CMD_DONE,
    OUT_CMD_DONE_OFF,
    OUT_CMD_HELP,
    OUT_CMD_HELP_OFF,
    OUT_CMD_TIME,
    OUT_CMD_REQUEST,
    OUT_CMD_STATUS,
    OUT_CMD_ERR,
    OUT_CMD_METRIC,
    OUT_CMD_ACK,
    OUT_CMD_LOC,
    OUT_CMD_MAX
} protocol_cmd_outbound_t;

typedef struct {
    protocol_cmd_inbound_t type;
    char node_id[4];
    uint32_t msg_id;
    int value;       
    char payload[128];  // optional additional data
} protocol_parsed_msg_t;

/* =========================
 * Message builders
 * ========================= */
void proto_hello(char *out, size_t len, const char *node_id, const char *msgid);
void proto_ver(char *out, size_t len, const char *node_id, const char *fw_ver, const char *msgid);
void proto_ping(char *out, size_t len, const char *node_id, const char *msgid);
void proto_ping_rssi(char *out, size_t len, const char *node_id, const char *msgid, int rssi);

void proto_done(char *out, size_t len, const char *node_id, const char *msgid);
void proto_done_off(char *out, size_t len, const char *node_id, const char *msgid);

void proto_help(char *out, size_t len, const char *node_id, const char *msgid);
void proto_help_off(char *out, size_t len, const char *node_id, const char *msgid);

void proto_time(char *out, size_t len, const char *node_id, const char *minutes, const char *msgid);
void proto_request(char *out, size_t len, const char *node_id, const char *msgid);
void proto_status(char *out, size_t len, const char *node_id, const char *code, const char *msgid);
void proto_err(char *out, size_t len, const char *node_id,const char *err_code, const char *msgid);

void proto_metric(char *out, size_t len, const char *node_id,const char *key, const char *value, const char *msgid);

void proto_ack(char *out, size_t len, const char *type,const char *node_id, const char *msgid);

void proto_loc(char *out, size_t len, const char *node_id,const char *beacon_id, int rssi, const char *msgid);

const char *proto_cmd_inbound_to_str(protocol_cmd_inbound_t cmd);
const char *proto_cmd_outbound_to_str(protocol_cmd_outbound_t cmd);
bool parse_inbound_message(const char *input, protocol_parsed_msg_t *out);

