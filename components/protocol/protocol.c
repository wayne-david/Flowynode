#include "protocol.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <stdbool.h>

/* =========================
 * Inbound Command enum (for parsing)
 * ========================= */

const char *protocol_cmd_inbound_str[IN_CMD_MAX] = {
    [IN_CMD_UNKNOWN]  = "UNKNOWN",
    [IN_CMD_HELLO_ACK]  = "HELLO_ACK",
    [IN_CMD_TSET]     = "TSET",
    [IN_CMD_TGO]      = "TGO",
    [IN_CMD_TSTOP]    = "TSTOP",
    [IN_CMD_TSYNC]    = "TSYNC",
    [IN_CMD_TADD]     = "TADD",
    [IN_CMD_CLEAR_OFFSET] = "CLEAR_OFFSET",
    [IN_CMD_CLEAR_OFFSET_ALL] = "CLEAR_OFFSET_ALL",
    [IN_CMD_STATUS_SET] = "STATUS_SET",
    [IN_CMD_STATUS_CLEAR] = "STATUS_CLEAR",
    [IN_CMD_STATUS_CLEAR_ALL] = "STATUS_CLEAR_ALL",
    [IN_CMD_RESET]    = "RESET",
    [IN_CMD_RESET_ALL] = "RESET_ALL",
    [IN_CMD_SLEEP]    = "SLEEP",
    [IN_CMD_WAKE]     = "WAKE",
    [IN_CMD_ACK]      = "ACK",
    [IN_CMD_CFG_BEACONS] = "CFG_BEACONS"
};

const char *protocol_cmd_outbound_str[OUT_CMD_MAX] = {
    [OUT_CMD_UNKNOWN]  = "UNKNOWN",
    [OUT_CMD_HELLO]    = "HELLO",
    [OUT_CMD_VER]      = "VER",
    [OUT_CMD_PING]     = "PING",
    [OUT_CMD_DONE]     = "DONE",
    [OUT_CMD_DONE_OFF] = "DONE_OFF",
    [OUT_CMD_HELP]     = "HELP",
    [OUT_CMD_HELP_OFF] = "HELP_OFF",
    [OUT_CMD_TIME]     = "TIME",
    [OUT_CMD_REQUEST]  = "REQUEST",
    [OUT_CMD_STATUS]   = "STATUS",
    [OUT_CMD_ERR]      = "ERR",
    [OUT_CMD_METRIC]   = "METRIC",
    [OUT_CMD_ACK]      = "ACK",
    [OUT_CMD_LOC]      = "LOC"
};

/* =========================
 * Outbound Message builders
 * ========================= */

void proto_hello(char *out, size_t len, const char *node_id, const char * msgid)
{
    snprintf(out, len, "ID:HELLO:%s:%s", node_id, msgid);
}

void proto_ver(char *out, size_t len, const char *node_id,const char *fw_ver, const char * msgid)
{
    snprintf(out, len, "ID:VER:%s:%s:%s", node_id, fw_ver, msgid);
}

void proto_ping(char *out, size_t len, const char *node_id, const char * msgid)
{
    snprintf(out, len, "ID:PING:%s:%s", node_id, msgid);
}

void proto_ping_rssi(char *out, size_t len, const char *node_id,const char * msgid, int rssi)
{
    snprintf(out, len, "ID:PING:%s:%s:RSSI:%d", node_id, msgid, rssi);
}

void proto_done(char *out, size_t len, const char *node_id, const char * msgid)
{
    snprintf(out, len, "ID:DONE:%s:%s", node_id, msgid);
}

void proto_done_off(char *out, size_t len, const char *node_id, const char * msgid)
{
    snprintf(out, len, "ID:DONE_OFF:%s:%s", node_id, msgid);
}

void proto_help(char *out, size_t len, const char *node_id, const char * msgid)
{
    snprintf(out, len, "ID:HELP:%s:%s", node_id, msgid);
}

void proto_help_off(char *out, size_t len, const char *node_id, const char * msgid)
{
    snprintf(out, len, "ID:HELP_OFF:%s:%s", node_id, msgid);
}

void proto_time(char *out, size_t len, const char *node_id,const char * minutes, const char * msgid)
{
    snprintf(out, len, "ID:TIME:%s:%s:%s", node_id, minutes, msgid);
}

void proto_request(char *out, size_t len, const char *node_id, const char * msgid)
{
    snprintf(out, len, "ID:REQUEST:%s:%s", node_id, msgid);
}

void proto_status(char *out, size_t len, const char *node_id,const char *code, const char * msgid)
{
    snprintf(out, len, "ID:STATUS:%s:%s:%s", node_id, code, msgid);
}

void proto_err(char *out, size_t len, const char *node_id,const char *err_code, const char * msgid)
{
    snprintf(out, len, "ID:ERR:%s:%s:%s", node_id, err_code, msgid);
}

void proto_metric(char *out, size_t len, const char *node_id,const char *key, const char *value, const char * msgid)
{
    snprintf(out, len, "ID:METRIC:%s:%s:%s:%s", node_id, key, value, msgid);
}

void proto_ack(char *out, size_t len, const char *type,const char *node_id, const char * msgid)
{
    snprintf(out, len, "ACK:%s:%s:%s", type, node_id, msgid);
}

void proto_loc(char *out, size_t len, const char *node_id,const char *beacon_id, int rssi, const char * msgid)
{
    snprintf(out, len, "LOC:%s:%s:%d:%s", node_id, beacon_id, rssi, msgid);
}

/* =========================
 * Command detection
 * ========================= */

const char *proto_cmd_inbound_to_str(protocol_cmd_inbound_t cmd)
{
    if (cmd >= IN_CMD_MAX)
        return "UNKNOWN";

    return protocol_cmd_inbound_str[cmd];
}

const char *proto_cmd_outbound_to_str(protocol_cmd_outbound_t cmd)
{
    if (cmd >= OUT_CMD_MAX)
        return "UNKNOWN";

    return protocol_cmd_outbound_str[cmd];
}

static char *next_tok(char **ctx) {
    return strtok_r(NULL, ":", ctx);
}

bool parse_inbound_message(const char *input, protocol_parsed_msg_t *out)
{
    if (!input || !out) return false;

    memset(out, 0, sizeof(*out));
    out->node_id[0] = '\0';
    out->msg_id  = -1;
    out->value   = -1;

    char buf[PROTO_MAX_MSG_LEN];
    strncpy(buf, input, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = 0;

    char *ctx;
    char *tok = strtok_r(buf, ":", &ctx);
    if (!tok) return false;

    if (strcmp(tok, "HELLO_ACK") == 0) {
        out->type = IN_CMD_HELLO_ACK;
        tok = next_tok(&ctx);
        if (tok) {
            strncpy(out->node_id, tok, sizeof(out->node_id) - 1);
            out->node_id[sizeof(out->node_id) - 1] = '\0';
        }
        return true;
    }

    if (strcmp(tok, "TSET") == 0) {
        out->type = IN_CMD_TSET;
        tok = next_tok(&ctx);
        if (!tok) return false;
        out->value = atoi(tok);
        return true;
    }

    if (strcmp(tok, "TGO") == 0) {
        out->type = IN_CMD_TGO;
        return true;
    }

    if (strcmp(tok, "TSTOP") == 0) {
        out->type = IN_CMD_TSTOP;
        return true;
    }

    if (strcmp(tok, "TSYNC") == 0) {
        out->type = IN_CMD_TSYNC;
        tok = next_tok(&ctx);
        if (!tok) return false;
        out->value = atoi(tok);
        return true;
    }

    if (strcmp(tok, "TADD") == 0) {
        out->type = IN_CMD_TADD;
        tok = next_tok(&ctx);
        if (!tok) return false;
        if (tok) {
            strncpy(out->node_id, tok, sizeof(out->node_id) - 1);
            out->node_id[sizeof(out->node_id) - 1] = '\0';
        }
        tok = next_tok(&ctx);
        if (!tok) return false;
        out->value = atoi(tok);
        return true;
    }

    if (strcmp(tok, "RESET_ALL") == 0) {
        out->type = IN_CMD_RESET_ALL;
        return true;
    }

    if (strcmp(tok, "RESET") == 0) {
        out->type = IN_CMD_RESET;
        tok = next_tok(&ctx);
        if (!tok) return false;
        if (tok) {
            strncpy(out->node_id, tok, sizeof(out->node_id) - 1);
            out->node_id[sizeof(out->node_id) - 1] = '\0';
        }
        return true;
    }

    if (strcmp(tok, "ACK") == 0) {
        out->type = IN_CMD_ACK;
        /* ACK:<TYPE>:<NODE_ID>:<MSGID> */
        tok = next_tok(&ctx); // TYPE
        if (!tok) return false;

        tok = next_tok(&ctx); // NODE
        if (!tok) return false;
        if (tok) {
            strncpy(out->node_id, tok, sizeof(out->node_id) - 1);
            out->node_id[sizeof(out->node_id) - 1] = '\0';
        }

        tok = next_tok(&ctx); // MSGID
        if (!tok) return false;
        out->msg_id = atoi(tok);
        return true;
    }

    if (strcmp(tok, "CFG_BEACONS") == 0) {
        out->type = IN_CMD_CFG_BEACONS;
        /* VERSION already consumed, rest is payload */
        strncpy(out->payload, ctx, sizeof(out->payload) - 1);
        return true;
    }
    return false;
}



