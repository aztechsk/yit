/*
 * yit.c
 *
 * Copyright (c) 2020 Jan Rusnak <jan@rusnak.sk>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <gentyp.h>
#include "sysconf.h"
#include "board.h"
#include "msgconf.h"
#include "criterr.h"
#include "yit_cmd.h"
#include "tools.h"
#include "yit.h"
#include <stdio.h>

#if YIT_LOG_COMMANDS == 1 && TERMOUT != 1
  #error "YIT_LOG_COMMANDS == 1 && TERMOUT != 1"
#endif

enum yit_par_tbl_type {
	YIT_SERIAL_NUM_TABLE = 0x05,
	YIT_CONF_PARAM_TABLE = 0x06,
	YIT_FACT_DEFAULT_TABLE = 0x08,
	YIT_FACT_DEFAULT_VALID_TABLE = 0x09,
	YIT_DEBUG_COUNTERS_TABLE = 0x0B,
	YIT_ALL_TABLES = 0xFF
};

enum yit_cfg_par_tbl_idx {
	YIT_MODULATION_IDX = 0x001A,
	YIT_OP_MODE_IDX = 0x0031,
	YIT_NET_SIZE_IDX = 0x0038,
	YIT_OP_BAND_IDX = 0x005F,
	YIT_MAX_NET_DEPTH_IDX = 0x0072,
	YIT_AUTO_CFG_ENAB_IDX = 0x0100,
        YIT_RX_FILT_M_IDX = 0x0107,
	YIT_RX_FILT_D_IDX = 0x0108,
	YIT_RX_FILT_N_IDX = 0x0109,
	YIT_RX_FILT_I_IDX = 0x010A,
	YIT_RX_FILT_C_IDX = 0x010B,
	YIT_RX_FILT_MR_IDX = 0x010D,
	YIT_RX_FILT_OR_IDX = 0x010E,
	YIT_NL_MNG_ENAB_IDX = 0x0200,
	YIT_WARM_START_ENAB_IDX = 0x0202,
	YIT_PARENT_MODE_ENAB_IDX = 0x0203,
        YIT_NET_ID_SEL_MODE_IDX = 0x0206,
	YIT_REM_CFG_ENAB_IDX = 0x020D,
	YIT_REM_VER_DOWN_ENAB_IDX = 0x020E
};

enum yit_rst_state {
	YIT_RST_NO_EEPROM = 6,
	YIT_RST_SUCCESS = 7,
	YIT_RST_FACTORY_DEFULT = 8,
	YIT_RST_DLL_FATAL_ERROR = 32,
	YIT_RST_DLL_AUTO_ONLINE_MODE = 64,
	YIT_RST_DLL_SAFE_MODE = 66
};

enum yit_disc_reason {
	YIT_DISC_PARENT_UNSTABLE,
	YIT_DISC_NVR_NACK,
	YIT_DISC_INFINITY,
	YIT_DISC_INIT,
	YIT_DISC_CANT_START_TIMER,
	YIT_DISC_NVR_REFUSED,
	YIT_DISC_NVR_ENQ,
	YIT_DISC_INVALID_NODE_ID,
	YIT_DISC_BY_APP_REQ = 10
};

enum yit_adm_deny_reason {
	YIT_ADM_DENY_DB_FULL = 1,
	YIT_ADM_DENY_BY_APP,
	YIT_ADM_DENY_NO_REASON,
	YIT_ADM_DENY_SN_NOT_IN_RANGE,
	YIT_ADM_DENY_DUP_NODE_ID,
	YIT_ADM_DENY_WRONG_NODE_ID,
	YIT_ADM_DENY_WRONG_CONFIRM_KEY
};

enum yit_tx_adm_result {
	YIT_ADM_RESULT_ACCEPTED,
	YIT_ADM_RESULT_REJECTED_NO_MEMORY,
	YIT_ADM_RESULT_REJECTED_FATAL_ERROR,
	YIT_ADM_RESULT_REJECTED_NODE_NOT_FOUND
};

enum yit_tx_trans_result {
	YIT_TRANS_RESULT_TRANSMITTED,
	YIT_TRANS_RESULT_N_A,
	YIT_TRANS_RESULT_NO_ACKNOWLEDGE,
	YIT_TRANS_RESULT_NO_TARGET_RESOURCES,
	YIT_TRANS_RESULT_BLOCKED,
	YIT_TRANS_RESULT_UNKNOWN_ERROR
};

#define YIT_GEN_RESP_TMO (20000 / portTICK_PERIOD_MS)
#define YIT_RST_RESP_TMO (3000 / portTICK_PERIOD_MS)
#define YIT_TXPKT_RESP_TMO (3000 / portTICK_PERIOD_MS)
#define YIT_NOOP_TMO (60000 / portTICK_PERIOD_MS)

uint8_t yit_snd_buf[YIT_SND_BUF_SIZE];

typedef void (*gfp_t)(void);
typedef gfp_t (*stfp_t)(void);

struct cfg_par {
	enum yit_cfg_par_tbl_idx idx;
	uint16_t par;
};

static const struct cfg_par cfg_par_ary[] = {
	{YIT_MODULATION_IDX, 0},
	{YIT_OP_MODE_IDX, 0},
	{YIT_NET_SIZE_IDX, YIT_NETWORK_SIZE},
	{YIT_MAX_NET_DEPTH_IDX, YIT_MAX_NETWORK_DEPTH},
	{YIT_AUTO_CFG_ENAB_IDX, 0},
	{YIT_RX_FILT_M_IDX, 1},
	{YIT_RX_FILT_D_IDX, 0},
	{YIT_RX_FILT_N_IDX, 0},
	{YIT_RX_FILT_I_IDX, 0},
	{YIT_RX_FILT_C_IDX, 0},
	{YIT_RX_FILT_MR_IDX, 0},
	{YIT_RX_FILT_OR_IDX, 0},
	{YIT_NL_MNG_ENAB_IDX, 1},
	{YIT_WARM_START_ENAB_IDX, YIT_WARM_START_ENABLED},
        {YIT_PARENT_MODE_ENAB_IDX, 1},
	{YIT_NET_ID_SEL_MODE_IDX, 0},
	{YIT_REM_CFG_ENAB_IDX, 1},
	{YIT_REM_VER_DOWN_ENAB_IDX, 1}
};

static void *ser_dev;

static void (*hw_rst_clbk)(void);
static uint16_t (*rx_intra_clbk)(struct yit_cmd *p_r);
static uint16_t (*rx_inter_clbk)(struct yit_cmd *p_r);
static void (*lv_net_clbk)(void);
static void (*no_data_clbk)(void);
static void (*led_ctl_clbk)(enum yit_led_ctl md);
static int (*ser_dev_snd)(void *ser_dev, void *buf, int sz);
static struct yit_cmd *(*ser_dev_rcv)(void *ser_dev, TickType_t tmo);
static void (*inval_yit_cmd)(struct yit_cmd *p_r);

static TaskHandle_t tsk_hndl;
static const char *tsk_nm = "YIT";
static stfp_t stmf;
static boolean_t save_par;
static uint8_t sn[16];
static uint16_t adm_tag;
static struct yit_stats stats;
static struct yit_it700_fw_ver fw_ver;
static boolean_t noop;
static struct yit_cmd *rxp_dfr;
static boolean_t link_on;
static short noop_cnt;
static boolean_t do_lv_net;

#if YIT_LOG_COMMANDS == 1
static const struct txt_item disc_reason_dsc_ary[] = {
	{YIT_DISC_PARENT_UNSTABLE, "PARENT_UNSTABLE"},
	{YIT_DISC_NVR_NACK, "NVR_NACK"},
	{YIT_DISC_INFINITY, "INFINITY"},
	{YIT_DISC_INIT, "INIT"},
	{YIT_DISC_CANT_START_TIMER, "CANT_START_TIMER"},
	{YIT_DISC_NVR_REFUSED, "NVR_REFUSED"},
	{YIT_DISC_NVR_ENQ, "NVR_ENQ"},
	{YIT_DISC_INVALID_NODE_ID, "INVALID_NODE_ID"},
	{YIT_DISC_BY_APP_REQ, "BY_APP_REQ"},
	{0, NULL}
};

static const char tx_packet_str[] = "TX_PACKET";

static const struct txt_item cmd_dsc_ary[] = {
	{YIT_NO_OPERATION, "NO_OPERATION"},
	{YIT_GET_VERSION, "GET_VERSION"},
	{YIT_GET_FREE_MEMORY, "GET_FREE_MEMORY"},
	{YIT_READ_FROM_NVM, "READ_FROM_NVM"},
	{YIT_WRITE_TO_NVM, "WRITE_TO_NVM"},
	{YIT_RESET, "RESET"},
	{YIT_GO_ONLINE, "GO_ONLINE"},
	{YIT_GO_OFFLINE, "GO_OFFLINE"},
	{YIT_SET_PREDEFINED_PARAMETERS, "SET_PREDEFINED_PARAMETERS"},
	{YIT_SET_DEVICE_PARAMETERS, "SET_DEVICE_PARAMETERS"},
	{YIT_GET_DEVICE_PARAMETERS, "GET_DEVICE_PARAMETERS"},
	{YIT_SAVE_DEVICE_PARAMETERS, "SAVE_DEVICE_PARAMETERS"},
	{YIT_REMOTE_PARAMETERS_CHANGED, "REMOTE_PARAMETERS_CHANGED"},
	{YIT_TX_PACKET, tx_packet_str},
	{YIT_GET_NC_DATABASE_SIZE, "GET_NC_DATABASE_SIZE"},
	{YIT_RX_PACKET, "RX_PACKET"},
	{YIT_GET_NODE_INFORMATION, "GET_NODE_INFORMATION"},
	{YIT_DELETE_NODE_INFORMATION, "DELETE_NODE_INFORMATION"},
	{YIT_ADMISSION_APPROVAL_RESP_FROM_APP, "ADMIS_APPR_RESP_FROM_APP"},
	{YIT_LEAVE_NETWORK, "LEAVE_NETWORK"},
	{YIT_CONNECTIVITY_STATUS_WITH_RS, "CONNECT_STATUS_WITH_RS"},
	{YIT_NODE_LEFT_NETWORK, "NODE_LEFT_NETWORK"},
	{YIT_GET_ADMISSION_APPROVAL_FROM_APP, "GET_ADMIS_APPR_FROM_APP"},
	{YIT_ADMISSION_REFUSE, "ADMISSION_REFUSE"},
	{YIT_CONNECTED_TO_NC, "CONNECTED_TO_NC"},
	{YIT_DISCONNECTED_FROM_NC, "DISCONNECTED_FROM_NC"},
	{YIT_NEW_CONNECTION_TO_NC, "NEW_CONNECTION_TO_NC"},
	{YIT_NETWORK_ID_ASSIGNED, "NETWORK_ID_ASSIGNED"},
	{0, NULL}
};

static const struct txt_item adm_deny_dsc_ary[] = {
	{YIT_ADM_DENY_DB_FULL, "DB_FULL"},
	{YIT_ADM_DENY_BY_APP, "BY_APP"},
	{YIT_ADM_DENY_NO_REASON, "NO_REASON"},
	{YIT_ADM_DENY_SN_NOT_IN_RANGE, "SN_NOT_IN_RANGE"},
	{YIT_ADM_DENY_DUP_NODE_ID, "DUP_NODE_ID"},
	{YIT_ADM_DENY_WRONG_NODE_ID, "WRONG_NODE_ID"},
	{YIT_ADM_DENY_WRONG_CONFIRM_KEY, "WRONG_CONFIRM_KEY"},
	{0, NULL}
};

static const struct txt_item adm_result_dsc_ary[] = {
	{YIT_ADM_RESULT_ACCEPTED, "ACCEPTED"},
	{YIT_ADM_RESULT_REJECTED_NO_MEMORY, "REJECTED_NO_MEMORY"},
	{YIT_ADM_RESULT_REJECTED_FATAL_ERROR, "REJECTED_FATAL_ERROR"},
	{YIT_ADM_RESULT_REJECTED_NODE_NOT_FOUND, "REJECTED_NODE_NOT_FOUND"},
	{0, NULL}
};

static const struct txt_item trans_result_dsc_ary[] = {
	{YIT_TRANS_RESULT_TRANSMITTED, "TRANSMITTED"},
	{YIT_TRANS_RESULT_N_A, "N/A"},
	{YIT_TRANS_RESULT_NO_ACKNOWLEDGE, "NO_ACKNOWLEDGE"},
	{YIT_TRANS_RESULT_NO_TARGET_RESOURCES, "NO_TARGET_RESOURCES"},
	{YIT_TRANS_RESULT_BLOCKED, "BLOCKED"},
	{YIT_TRANS_RESULT_UNKNOWN_ERROR, "UNKNOWN_ERROR"},
	{0, NULL}
};

static const struct txt_item rx_pkt_type_dsc_ary[] = {
	{YIT_PKT_TYPE_TO_DEV_IN_OTHER_NET, "ONT"},
        {YIT_PKT_TYPE_TO_DEV_IN_MY_NET, "MNT"},
        {YIT_PKT_TYPE_TO_ME, "4ME"},
        {YIT_PKT_TYPE_RETRANSMITTED, "RET"},
        {YIT_PKT_TYPE_SPOOFED, "SPF"},
	{YIT_PKT_TYPE_MALFORMED, "MLF"},
	{0, NULL}
};
#endif

#if TERMOUT == 1
static const struct txt_item rst_state_dsc_ary[] = {
	{YIT_RST_NO_EEPROM, "NO_EEPROM"},
	{YIT_RST_SUCCESS, "SUCCESS"},
	{YIT_RST_FACTORY_DEFULT, "FACTORY_DEFULT"},
	{YIT_RST_DLL_FATAL_ERROR, "DLL_FATAL_ERROR"},
	{YIT_RST_DLL_AUTO_ONLINE_MODE, "DLL_AUTO_ONLINE_MODE"},
	{YIT_RST_DLL_SAFE_MODE, "DLL_SAFE_MODE"},
	{0, NULL}
};
#endif

#if TERMOUT == 1
static const char req_str[] = "<c";
static const char res_str[] = "r>";
static const char ind_str[] = "i>";
static const char *const cmd_type_arr[] = {req_str, res_str, ind_str};
#endif

static void yit_tsk(void *p);
static gfp_t state_hw_reset(void);
static gfp_t state_soft_reset(void);
static gfp_t state_read_sn(void);
static gfp_t state_read_fw_ver(void);
static gfp_t state_config_op_band(void);
static gfp_t state_config_param(void);
static gfp_t state_save_param(void);
static gfp_t state_go_online(void);
static gfp_t state_online(void);
static gfp_t state_tx_resp1(void);
static gfp_t rx_packet(struct yit_cmd *p_r);
static void yit_lv_net(void);
static boolean_t valid_resp(enum yit_opc opc, struct yit_cmd *p_r);
static void inval_rxp_dfr(void);
static void to_lit_en(void *p, uint16_t n);
static uint16_t fr_lit_en(void *p);
#if YIT_LOG_COMMANDS == 1
static void log_rcv_cmd(struct yit_cmd *p);
static void log_rx_pkt(struct yit_cmd *p);
#endif
#if TERMOUT == 1
static void log_sn(void);
static void log_fw_ver(void);
#endif

/**
 * init_yit_c
 */
void init_yit_c(void *ser_dev_inst,
		void (*hw_rst_clbk_fn)(void),
		uint16_t (*rx_intra_clbk_fn)(struct yit_cmd *p_r),
		uint16_t (*rx_inter_clbk_fn)(struct yit_cmd *p_r),
		void (*lv_net_clbk_fn)(void),
		void (*no_data_clbk_fn)(void),
		void (*led_ctl_clbk_fn)(enum yit_led_ctl md),
		int (*ser_dev_snd_fn)(void *ser_dev, void *buf, int sz),
		struct yit_cmd *(*ser_dev_rcv_fn)(void *ser_dev, TickType_t tmo),
		void (*inval_yit_cmd_fn)(struct yit_cmd *p_r))
{
	ser_dev = ser_dev_inst;
	hw_rst_clbk = hw_rst_clbk_fn;
	rx_intra_clbk = rx_intra_clbk_fn;
	rx_inter_clbk = rx_inter_clbk_fn;
	lv_net_clbk = lv_net_clbk_fn;
	no_data_clbk = no_data_clbk_fn;
	led_ctl_clbk = led_ctl_clbk_fn;
	ser_dev_snd = ser_dev_snd_fn;
	ser_dev_rcv = ser_dev_rcv_fn;
        inval_yit_cmd = inval_yit_cmd_fn;
	if (pdPASS != xTaskCreate(yit_tsk, tsk_nm, YIT_TASK_STACK_SIZE, NULL,
				  YIT_TASK_PRIO, &tsk_hndl)) {
		crit_err_exit(MALLOC_ERROR);
	}
}

/**
 * yit_stats
 */
struct yit_stats *yit_stats(void)
{
	return (&stats);
}

/**
 * yit_it700_fw_ver
 */
struct yit_it700_fw_ver *yit_it700_fw_ver(void)
{
	return (&fw_ver);
}

/**
 * yit_force_leave_net
 */
void yit_force_leave_net(void)
{
	do_lv_net = TRUE;
	led_ctl_clbk(YIT_LED_BLINK_FAST);
}

/**
 * yit_tsk
 */
static void yit_tsk(void *p)
{
	if (!hw_rst_clbk) {
		crit_err_exit(BAD_PARAMETER);
	}
	stmf = state_hw_reset;
	vTaskDelay(YIT_INIT_DELAY / portTICK_PERIOD_MS);
	while (TRUE) {
		stmf = (stfp_t) (*stmf)();
	}
}

/**
 * state_hw_reset
 */
static gfp_t state_hw_reset(void)
{
	struct yit_cmd *p_r;
	int i;

	inval_rxp_dfr();
	led_ctl_clbk(YIT_LED_OFF);
	for (i = 0; i < YIT_HW_RTS_TRY_CNT; i++) {
		stats.hw_rst++;
		hw_rst_clbk();
		if (NULL == (p_r = ser_dev_rcv(ser_dev, YIT_RST_RESP_TMO))) {
			msg(INF, "IT7# HW_RESET timeout\n");
			continue;
		}
#if YIT_LOG_COMMANDS == 1
		log_rcv_cmd(p_r);
#endif
		if (p_r->buf[0] != YIT_RESPONSE || p_r->buf[1] != YIT_RESET) {
			msg(INF, "IT7# HW_RESET error\n");
			inval_yit_cmd(p_r);
			continue;
		}
		msg(INF, "IT7# hw_init=%s\n",
		    find_txt_item(p_r->buf[2], rst_state_dsc_ary, "IDX_ERR"));
		if (p_r->buf[2] == YIT_RST_NO_EEPROM || p_r->buf[2] == YIT_RST_DLL_FATAL_ERROR) {
			inval_yit_cmd(p_r);
			continue;
		}
		break;
	}
	if (i == YIT_HW_RTS_TRY_CNT) {
		msg(INF, "IT7# device not found or broken\n");
		vTaskSuspend(NULL);
	}
	inval_yit_cmd(p_r);
	return ((gfp_t) state_read_sn);
}

/**
 * state_soft_reset
 */
static gfp_t state_soft_reset(void)
{
	struct yit_cmd *p_r;

	inval_rxp_dfr();
	led_ctl_clbk(YIT_LED_OFF);
        stats.sw_rst++;
	yit_snd_buf[0] = 0xCA;
	yit_snd_buf[1] = 0x02;
	yit_snd_buf[2] = 0x00;
	yit_snd_buf[3] = YIT_REQUEST;
	yit_snd_buf[4] = YIT_RESET;
	yit_sum_snd_buf();
#if YIT_LOG_COMMANDS == 1
	yit_log_snd_cmd();
#endif
	ser_dev_snd(ser_dev, yit_snd_buf, 6);
	if (NULL == (p_r = ser_dev_rcv(ser_dev, YIT_RST_RESP_TMO))) {
		msg(INF, "IT7# SOFT_RESET timeout\n");
		return ((gfp_t) state_hw_reset);
	}
#if YIT_LOG_COMMANDS == 1
	log_rcv_cmd(p_r);
#endif
	if (p_r->buf[0] != YIT_RESPONSE || p_r->buf[1] != YIT_RESET) {
		msg(INF, "IT7# SOFT_RESET error\n");
		inval_yit_cmd(p_r);
		return ((gfp_t) state_hw_reset);
	}
	msg(INF, "IT7# soft_init=%s\n",
            find_txt_item(p_r->buf[2], rst_state_dsc_ary, "IDX_ERR"));
	inval_yit_cmd(p_r);
	return ((gfp_t) state_config_op_band);
}

/**
 * state_read_sn
 */
static gfp_t state_read_sn(void)
{
	struct yit_cmd *p_r;

	yit_snd_buf[0] = 0xCA;
	yit_snd_buf[1] = 0x07;
	yit_snd_buf[2] = 0x00;
	yit_snd_buf[3] = YIT_REQUEST;
	yit_snd_buf[4] = YIT_GET_DEVICE_PARAMETERS;
	yit_snd_buf[5] = YIT_SERIAL_NUM_TABLE;
	to_lit_en(&yit_snd_buf[6], 0xBAAB);
	to_lit_en(&yit_snd_buf[8], 0);
	yit_sum_snd_buf();
#if YIT_LOG_COMMANDS == 1
	yit_log_snd_cmd();
#endif
	ser_dev_snd(ser_dev, yit_snd_buf, 11);
	if (NULL == (p_r = ser_dev_rcv(ser_dev, YIT_GEN_RESP_TMO))) {
		msg(INF, "IT7# GET_DEVICE_PARAMETERS timeout\n");
		return ((gfp_t) state_hw_reset);
	}
#if YIT_LOG_COMMANDS == 1
	log_rcv_cmd(p_r);
#endif
	if (!valid_resp(YIT_GET_DEVICE_PARAMETERS, p_r)) {
		inval_yit_cmd(p_r);
		return ((gfp_t) state_hw_reset);
	}
	for (int i = 0; i < 16; i++) {
		sn[i] = p_r->buf[i + 3];
	}
#if TERMOUT == 1
	log_sn();
#endif
	inval_yit_cmd(p_r);
	return ((gfp_t) state_read_fw_ver);
}

/**
 * state_read_fw_ver
 */
static gfp_t state_read_fw_ver(void)
{
	struct yit_cmd *p_r;

	yit_snd_buf[0] = 0xCA;
	yit_snd_buf[1] = 0x02;
	yit_snd_buf[2] = 0x00;
	yit_snd_buf[3] = YIT_REQUEST;
	yit_snd_buf[4] = YIT_GET_VERSION;
        yit_snd_buf[5] = 0x03;
#if YIT_LOG_COMMANDS == 1
	yit_log_snd_cmd();
#endif
	ser_dev_snd(ser_dev, yit_snd_buf, 6);
	if (NULL == (p_r = ser_dev_rcv(ser_dev, YIT_GEN_RESP_TMO))) {
		msg(INF, "IT7# GET_VERSION timeout\n");
		return ((gfp_t) state_hw_reset);
	}
#if YIT_LOG_COMMANDS == 1
	log_rcv_cmd(p_r);
#endif
	if (!valid_resp(YIT_GET_VERSION, p_r)) {
		inval_yit_cmd(p_r);
		return ((gfp_t) state_hw_reset);
	}
	fw_ver.maj = p_r->buf[3];
        fw_ver.min = p_r->buf[4];
        fw_ver.bld = p_r->buf[5];
#if TERMOUT == 1
	log_fw_ver();
#endif
	inval_yit_cmd(p_r);
	return ((gfp_t) state_config_op_band);
}

/**
 * state_config_op_band
 */
static gfp_t state_config_op_band(void)
{
	struct yit_cmd *p_r;

	save_par = FALSE;
	yit_snd_buf[0] = 0xCA;
	yit_snd_buf[1] = 0x07;
	yit_snd_buf[2] = 0x00;
	yit_snd_buf[3] = YIT_REQUEST;
	yit_snd_buf[4] = YIT_GET_DEVICE_PARAMETERS;
	yit_snd_buf[5] = YIT_CONF_PARAM_TABLE;
	to_lit_en(&yit_snd_buf[6], YIT_OP_BAND_IDX);
	to_lit_en(&yit_snd_buf[8], 1);
	yit_sum_snd_buf();
#if YIT_LOG_COMMANDS == 1
	yit_log_snd_cmd();
#endif
	ser_dev_snd(ser_dev, yit_snd_buf, 11);
	if (NULL == (p_r = ser_dev_rcv(ser_dev, YIT_GEN_RESP_TMO))) {
		msg(INF, "IT7# GET_DEVICE_PARAMETERS timeout\n");
		return ((gfp_t) state_hw_reset);
	}
#if YIT_LOG_COMMANDS == 1
	log_rcv_cmd(p_r);
#endif
	if (!valid_resp(YIT_GET_DEVICE_PARAMETERS, p_r)) {
		inval_yit_cmd(p_r);
		return ((gfp_t) state_soft_reset);
	}
	if (fr_lit_en(&p_r->buf[3]) == YIT_OPERATION_BAND) {
		inval_yit_cmd(p_r);
		return ((gfp_t) state_config_param);
	}
	inval_yit_cmd(p_r);
	yit_snd_buf[0] = 0xCA;
	yit_snd_buf[1] = 0x04;
	yit_snd_buf[2] = 0x00;
	yit_snd_buf[3] = YIT_REQUEST;
	yit_snd_buf[4] = YIT_SET_PREDEFINED_PARAMETERS;
	yit_snd_buf[5] = YIT_ALL_TABLES;
	switch (YIT_OPERATION_BAND) {
	case YIT_OP_BAND_FCC  :
		yit_snd_buf[6] = YIT_REGION_FCC;
		break;
	case YIT_OP_BAND_CA   :
		yit_snd_buf[6] = YIT_REGION_CA;
		break;
	case YIT_OP_BAND_CB   :
		yit_snd_buf[6] = YIT_REGION_CB;
		break;
	case YIT_OP_BAND_CA3  :
		yit_snd_buf[6] = YIT_REGION_CA3;
		break;
	case YIT_OP_BAND_ARIB :
		yit_snd_buf[6] = YIT_REGION_ARIB;
		break;
	default               :
		crit_err_exit(BAD_PARAMETER);
		break;
	}
	yit_sum_snd_buf();
#if YIT_LOG_COMMANDS == 1
	yit_log_snd_cmd();
#endif
	ser_dev_snd(ser_dev, yit_snd_buf, 8);
	if (NULL == (p_r = ser_dev_rcv(ser_dev, YIT_GEN_RESP_TMO))) {
		msg(INF, "IT7# SET_PREDEFINED_PARAMETERS timeout\n");
		return ((gfp_t) state_hw_reset);
	}
#if YIT_LOG_COMMANDS == 1
	log_rcv_cmd(p_r);
#endif
	(void) valid_resp(YIT_SET_PREDEFINED_PARAMETERS, p_r);
	save_par = TRUE;
	inval_yit_cmd(p_r);
	return ((gfp_t) state_config_param);
}

/**
 * state_config_param
 */
static gfp_t state_config_param(void)
{
	struct cfg_par cp;
	struct yit_cmd *p_r;

	for (size_t i = 0; i < sizeof(cfg_par_ary) / sizeof(struct cfg_par); i++) {
		cp = cfg_par_ary[i];
		yit_snd_buf[0] = 0xCA;
		yit_snd_buf[1] = 0x07;
		yit_snd_buf[2] = 0x00;
		yit_snd_buf[3] = YIT_REQUEST;
		yit_snd_buf[4] = YIT_GET_DEVICE_PARAMETERS;
		yit_snd_buf[5] = YIT_CONF_PARAM_TABLE;
		to_lit_en(&yit_snd_buf[6], cp.idx);
		to_lit_en(&yit_snd_buf[8], 1);
		yit_sum_snd_buf();
#if YIT_LOG_COMMANDS == 1
		yit_log_snd_cmd();
#endif
		ser_dev_snd(ser_dev, yit_snd_buf, 11);
		if (NULL == (p_r = ser_dev_rcv(ser_dev, YIT_GEN_RESP_TMO))) {
			msg(INF, "IT7# GET_DEVICE_PARAMETERS timeout\n");
			return ((gfp_t) state_hw_reset);
		}
#if YIT_LOG_COMMANDS == 1
		log_rcv_cmd(p_r);
#endif
		if (!valid_resp(YIT_GET_DEVICE_PARAMETERS, p_r)) {
			inval_yit_cmd(p_r);
			return ((gfp_t) state_soft_reset);
		}
		if (fr_lit_en(&p_r->buf[3]) == cp.par) {
			inval_yit_cmd(p_r);
			continue;
		}
		inval_yit_cmd(p_r);
		yit_snd_buf[0] = 0xCA;
		yit_snd_buf[1] = 0x07;
		yit_snd_buf[2] = 0x00;
		yit_snd_buf[3] = YIT_REQUEST;
		yit_snd_buf[4] = YIT_SET_DEVICE_PARAMETERS;
		yit_snd_buf[5] = YIT_CONF_PARAM_TABLE;
		to_lit_en(&yit_snd_buf[6], cp.idx);
		to_lit_en(&yit_snd_buf[8], cp.par);
		yit_sum_snd_buf();
#if YIT_LOG_COMMANDS == 1
		yit_log_snd_cmd();
#endif
		ser_dev_snd(ser_dev, yit_snd_buf, 11);
		if (NULL == (p_r = ser_dev_rcv(ser_dev, YIT_GEN_RESP_TMO))) {
			msg(INF, "IT7# SET_DEVICE_PARAMETERS timeout\n");
			return ((gfp_t) state_hw_reset);
		}
#if YIT_LOG_COMMANDS == 1
		log_rcv_cmd(p_r);
#endif
		if (!valid_resp(YIT_SET_DEVICE_PARAMETERS, p_r)) {
			inval_yit_cmd(p_r);
			return ((gfp_t) state_soft_reset);
		}
		inval_yit_cmd(p_r);
		save_par = TRUE;
	}
	if (save_par) {
		return ((gfp_t) state_save_param);
	} else {
		return ((gfp_t) state_go_online);
	}
}

/**
 * state_save_param
 */
static gfp_t state_save_param(void)
{
	struct yit_cmd *p_r;

	yit_snd_buf[0] = 0xCA;
	yit_snd_buf[1] = 0x03;
	yit_snd_buf[2] = 0x00;
	yit_snd_buf[3] = YIT_REQUEST;
	yit_snd_buf[4] = YIT_SAVE_DEVICE_PARAMETERS;
	yit_snd_buf[5] = YIT_CONF_PARAM_TABLE;
	yit_sum_snd_buf();
#if YIT_LOG_COMMANDS == 1
	yit_log_snd_cmd();
#endif
	ser_dev_snd(ser_dev, yit_snd_buf, 7);
	if (NULL == (p_r = ser_dev_rcv(ser_dev, YIT_GEN_RESP_TMO))) {
		msg(INF, "IT7# SAVE_DEVICE_PARAMETERS timeout\n");
		return ((gfp_t) state_hw_reset);
	}
#if YIT_LOG_COMMANDS == 1
	log_rcv_cmd(p_r);
#endif
	(void) valid_resp(YIT_SAVE_DEVICE_PARAMETERS, p_r);
	inval_yit_cmd(p_r);
	return ((gfp_t) state_soft_reset);
}

/**
 * state_go_online
 */
static gfp_t state_go_online(void)
{
	struct yit_cmd *p_r;

	for (size_t i = 0; i < sizeof(cfg_par_ary) / sizeof(struct cfg_par); i++) {
		if (cfg_par_ary[i].idx == YIT_WARM_START_ENAB_IDX) {
			if (cfg_par_ary[i].par == 0 || cfg_par_ary[i].par == 1) {
				msg(INF, "IT7# warm start %s\n",
			           (cfg_par_ary[i].par) ? "enabled" : "disabled");
			} else {
				msg(INF, "IT7# bad parameter WARM_START_ENABLED\n");
				crit_err_exit(BAD_PARAMETER);
			}
		}
	}
	yit_snd_buf[0] = 0xCA;
	yit_snd_buf[1] = 0x02;
	yit_snd_buf[2] = 0x00;
	yit_snd_buf[3] = YIT_REQUEST;
	yit_snd_buf[4] = YIT_GO_ONLINE;
	yit_sum_snd_buf();
#if YIT_LOG_COMMANDS == 1
	yit_log_snd_cmd();
#endif
	ser_dev_snd(ser_dev, yit_snd_buf, 6);
	if (NULL == (p_r = ser_dev_rcv(ser_dev, YIT_GEN_RESP_TMO))) {
		msg(INF, "IT7# GO_ONLINE timeout\n");
		return ((gfp_t) state_hw_reset);
	}
#if YIT_LOG_COMMANDS == 1
	log_rcv_cmd(p_r);
#endif
	if (!valid_resp(YIT_GO_ONLINE, p_r)) {
		inval_yit_cmd(p_r);
		return ((gfp_t) state_soft_reset);
	}
	link_on = FALSE;
	noop = FALSE;
	noop_cnt = 0;
	led_ctl_clbk(YIT_LED_BLINK_NORMAL);
	inval_yit_cmd(p_r);
	return ((gfp_t) state_online);
}

/**
 * state_online
 */
static gfp_t state_online(void)
{
	struct yit_cmd *p_r;
	gfp_t gfp;

	while (TRUE) {
		if (do_lv_net && !noop) {
			do_lv_net = FALSE;
			yit_lv_net();
		}
		if (NULL == (p_r = ser_dev_rcv(ser_dev, YIT_NOOP_TMO))) {
			if (noop) {
				msg(INF, "IT7# NO_OPERATION timeout\n");
				return ((gfp_t) state_hw_reset);
			} else {
				yit_snd_buf[0] = 0xCA;
				yit_snd_buf[1] = 0x02;
				yit_snd_buf[2] = 0x00;
				yit_snd_buf[3] = YIT_REQUEST;
				yit_snd_buf[4] = YIT_NO_OPERATION;
				yit_snd_buf[5] = 0x02;
#if YIT_LOG_COMMANDS == 1 && YIT_LOG_NOOP_COMMAND == 1
				yit_log_snd_cmd();
#endif
				ser_dev_snd(ser_dev, yit_snd_buf, 6);
				noop = TRUE;
				continue;
			}
		}
		switch (p_r->buf[1]) {
		case YIT_TX_PACKET :
			if (p_r->buf[0] != YIT_RESPONSE) {
				msg(INF, "IT7# TX_PACKET error\n");
				inval_yit_cmd(p_r);
                                return ((gfp_t) state_soft_reset);
			}
			if (p_r->buf[3] == 3) {
#if YIT_LOG_COMMANDS == 1
				msg(INF, "IT7# r> TX_PACKET2 %d %s\n", fr_lit_en(&p_r->buf[7]),
				    find_txt_item(p_r->buf[4], trans_result_dsc_ary, "IDX_ERR"));
#endif
				if (p_r->buf[4] != YIT_TRANS_RESULT_TRANSMITTED) {
					stats.txp_trn_err++;
				}
				inval_yit_cmd(p_r);
			} else if (p_r->buf[3] != 1) {
				msg(INF, "IT7# TX_PACKET response type error\n");
				inval_yit_cmd(p_r);
				return ((gfp_t) state_soft_reset);
			} else {
				msg(INF, "IT7# unexpected r> TX_PACKET1 %d in ONLINE state\n",
				    fr_lit_en(&p_r->buf[5]));
				inval_yit_cmd(p_r);
				return ((gfp_t) state_soft_reset);
			}
			break;
		case YIT_CONNECTED_TO_NC :
			if (p_r->buf[0] == YIT_INDICATION) {
				msg(INF, "IT7# i> CONNECTED_TO_NC, ParentID=%d\n",
				    fr_lit_en(&p_r->buf[2]));
				led_ctl_clbk(YIT_LED_ON);
				link_on = TRUE;
                                inval_yit_cmd(p_r);
			} else {
				msg(INF, "IT7# CONNECTED_TO_NC error\n");
				inval_yit_cmd(p_r);
				return ((gfp_t) state_soft_reset);
			}
			break;
		case YIT_DISCONNECTED_FROM_NC :
			if (p_r->buf[0] == YIT_INDICATION) {
#if YIT_LOG_COMMANDS == 1
				msg(INF, "IT7# i> DISCONNECTED_FROM_NC (%s)\n",
				    find_txt_item(p_r->buf[2], disc_reason_dsc_ary, "IDX_ERR"));
#else
				msg(INF, "IT7# i> DISCONNECTED_FROM_NC, reason=%d\n",
				    p_r->buf[2]);
#endif
				stats.disc_nc++;
				led_ctl_clbk(YIT_LED_BLINK_NORMAL);
				link_on = FALSE;
                                inval_yit_cmd(p_r);
			} else {
				msg(INF, "IT7# DISCONNECTED_FROM_NC error\n");
				inval_yit_cmd(p_r);
				return ((gfp_t) state_soft_reset);
			}
                        break;
		case YIT_ADMISSION_REFUSE :
			if (p_r->buf[0] == YIT_INDICATION) {
#if YIT_LOG_COMMANDS == 1
				msg(INF, "IT7# i> ADMISSION_REFUSE (%s)\n",
				    find_txt_item(fr_lit_en(&p_r->buf[2]) >> 12, adm_deny_dsc_ary, "IDX_ERR"));
#else
				msg(INF, "IT7# i> ADMISSION_REFUSE, reason=%x\n",
				    fr_lit_en(&p_r->buf[2]));
#endif
				if (link_on) {
					led_ctl_clbk(YIT_LED_BLINK_NORMAL);
					link_on = FALSE;
				}
                                inval_yit_cmd(p_r);
			} else {
				msg(INF, "IT7# ADMISSION_REFUSE error\n");
				inval_yit_cmd(p_r);
				return ((gfp_t) state_soft_reset);
			}
                        break;
		case YIT_NO_OPERATION :
#if YIT_LOG_COMMANDS == 1 && YIT_LOG_NOOP_COMMAND == 1
			log_rcv_cmd(p_r);
#endif
			if (!valid_resp(YIT_NO_OPERATION, p_r)) {
				inval_yit_cmd(p_r);
				return ((gfp_t) state_soft_reset);
			}
			noop = FALSE;
			inval_yit_cmd(p_r);
			if (rxp_dfr) {
				if ((gfp = rx_packet(rxp_dfr))) {
					inval_yit_cmd(rxp_dfr);
					rxp_dfr = NULL;
					return (gfp);
				} else {
					inval_yit_cmd(rxp_dfr);
					rxp_dfr = NULL;
                                        continue;
				}
			}
			if (noop_cnt++ == YIT_NO_DATA_TMO) {
				if (no_data_clbk) {
					(*no_data_clbk)();
				}
			} else if (noop_cnt == YIT_LEAVE_NET_TMO) {
				if (lv_net_clbk) {
					(*lv_net_clbk)();
				}
				yit_lv_net();
			}
			break;
		case YIT_RX_PACKET :
			if (p_r->buf[0] != YIT_INDICATION) {
				msg(INF, "IT7# RX_PACKET error\n");
				inval_yit_cmd(p_r);
				return ((gfp_t) state_soft_reset);
			}
                        if (p_r->buf[6] > 7 && p_r->buf[6] < 12 && !p_r->buf[3]) {
				noop_cnt = 0;
			}
			if (noop) {
				if (!rxp_dfr) {
					rxp_dfr = p_r;
				} else {
					msg(INF, "IT7# next i> RX_PACKET in NOOP sequence error\n");
                                        inval_yit_cmd(p_r);
                                        return ((gfp_t) state_soft_reset);
				}
			} else {
				if ((gfp = rx_packet(p_r))) {
					inval_yit_cmd(p_r);
					return (gfp);
				} else {
					inval_yit_cmd(p_r);
				}
			}
			break;
		case YIT_RESET :
			inval_rxp_dfr();
                        stats.wd_rst++;
#if YIT_LOG_COMMANDS == 1
			log_rcv_cmd(p_r);
#endif
			msg(INF, "IT7# wd_init=%s\n",
		            find_txt_item(p_r->buf[2], rst_state_dsc_ary, "IDX_ERR"));
			if (p_r->buf[2] == YIT_RST_NO_EEPROM || p_r->buf[2] == YIT_RST_DLL_FATAL_ERROR) {
				inval_yit_cmd(p_r);
				return ((gfp_t) state_hw_reset);
			} else {
				inval_yit_cmd(p_r);
				return ((gfp_t) state_config_op_band);
			}
                        break;
		case YIT_REMOTE_PARAMETERS_CHANGED :
			if (p_r->buf[0] != YIT_INDICATION) {
				msg(INF, "IT7# REMOTE_PARAMETERS_CHANGED error\n");
				inval_yit_cmd(p_r);
				return ((gfp_t) state_soft_reset);
			}
#if YIT_LOG_COMMANDS == 1
			log_rcv_cmd(p_r);
#endif
			inval_yit_cmd(p_r);
			break;
		default :
#if YIT_LOG_COMMANDS == 1
			msg(INF, "IT7# unexpected %s %s in ONLINE state\n",
			    cmd_type_arr[p_r->buf[0] & 0x07],
			    find_txt_item(p_r->buf[1], cmd_dsc_ary, "IDX_ERR"));
#else
			msg(INF, "IT7# unexpected %s %X in ONLINE state\n", cmd_type_arr[p_r->buf[0] & 0x07],
			    p_r->buf[1]);
#endif
			inval_yit_cmd(p_r);
                        return ((gfp_t) state_soft_reset);
		}
	}
}

/**
 * state_tx_resp1
 */
static gfp_t state_tx_resp1(void)
{
	struct yit_cmd *p_r;

	while (TRUE) {
		if (NULL == (p_r = ser_dev_rcv(ser_dev, YIT_TXPKT_RESP_TMO))) {
			msg(INF, "IT7# TX_PACKET1 timeout\n");
			return ((gfp_t) state_hw_reset);
		}
		switch (p_r->buf[1]) {
		case YIT_TX_PACKET :
			if (p_r->buf[0] != YIT_RESPONSE) {
				msg(INF, "IT7# TX_PACKET error\n");
				inval_yit_cmd(p_r);
                                return ((gfp_t) state_soft_reset);
			}
			if (p_r->buf[3] == 3) {
				msg(INF, "IT7# unexpected r> TX_PACKET2 %d\n", fr_lit_en(&p_r->buf[7]));
				inval_yit_cmd(p_r);
				return ((gfp_t) state_soft_reset);
			} else if (p_r->buf[3] != 1) {
				msg(INF, "IT7# TX_PACKET response type error\n");
				inval_yit_cmd(p_r);
				return ((gfp_t) state_soft_reset);
			}
			if (adm_tag != fr_lit_en(&p_r->buf[5])) {
				msg(INF, "IT7# TX_PACKET1 session tag %d not match with %d\n",
				    fr_lit_en(&p_r->buf[5]), adm_tag);
				inval_yit_cmd(p_r);
				return ((gfp_t) state_soft_reset);
			}
#if YIT_LOG_COMMANDS == 1
			msg(INF, "IT7# r> TX_PACKET1 %d %s\n", fr_lit_en(&p_r->buf[5]),
			    find_txt_item(p_r->buf[4], adm_result_dsc_ary, "IDX_ERR"));
#endif
			if (p_r->buf[4] != YIT_ADM_RESULT_ACCEPTED) {
				stats.txp_adm_err++;
			}
			inval_yit_cmd(p_r);
			return ((gfp_t) state_online);
		case YIT_CONNECTED_TO_NC :
			if (p_r->buf[0] == YIT_INDICATION) {
				msg(INF, "IT7# i> CONNECTED_TO_NC, ParentID=%d\n",
				    fr_lit_en(&p_r->buf[2]));
				led_ctl_clbk(YIT_LED_ON);
				link_on = TRUE;
				inval_yit_cmd(p_r);
			} else {
				msg(INF, "IT7# CONNECTED_TO_NC error\n");
				inval_yit_cmd(p_r);
				return ((gfp_t) state_soft_reset);
			}
                        break;
		case YIT_DISCONNECTED_FROM_NC :
			if (p_r->buf[0] == YIT_INDICATION) {
#if YIT_LOG_COMMANDS == 1
				msg(INF, "IT7# i> DISCONNECTED_FROM_NC (%s)\n",
				    find_txt_item(p_r->buf[2], disc_reason_dsc_ary, "IDX_ERR"));
#else
				msg(INF, "IT7# i> DISCONNECTED_FROM_NC, reason=%d\n",
				    p_r->buf[2]);
#endif
				stats.disc_nc++;
				led_ctl_clbk(YIT_LED_BLINK_NORMAL);
				link_on = FALSE;
				inval_yit_cmd(p_r);
			} else {
				msg(INF, "IT7# DISCONNECTED_FROM_NC error\n");
				inval_yit_cmd(p_r);
				return ((gfp_t) state_soft_reset);
			}
                        break;
		case YIT_ADMISSION_REFUSE :
			if (p_r->buf[0] == YIT_INDICATION) {
#if YIT_LOG_COMMANDS == 1
				msg(INF, "IT7# i> ADMISSION_REFUSE (%s)\n",
				    find_txt_item(fr_lit_en(&p_r->buf[2]) >> 12, adm_deny_dsc_ary, "IDX_ERR"));
#else
				msg(INF, "IT7# i> ADMISSION_REFUSE, reason=%x\n",
			            fr_lit_en(&p_r->buf[2]));
#endif
				if (link_on) {
					led_ctl_clbk(YIT_LED_BLINK_NORMAL);
					link_on = FALSE;
				}
				inval_yit_cmd(p_r);
			} else {
				msg(INF, "IT7# ADMISSION_REFUSE error\n");
				inval_yit_cmd(p_r);
				return ((gfp_t) state_soft_reset);
			}
                        break;
		case YIT_RX_PACKET :
			msg(INF, "IT7# i> RX_PACKET ignored in TX_RESP1 state\n");
			inval_yit_cmd(p_r);
			break;
		case YIT_RESET :
			inval_rxp_dfr();
                        stats.wd_rst++;
#if YIT_LOG_COMMANDS == 1
			log_rcv_cmd(p_r);
#endif
			msg(INF, "IT7# wd_init=%s\n",
		            find_txt_item(p_r->buf[2], rst_state_dsc_ary, "IDX_ERR"));
			if (p_r->buf[2] == YIT_RST_NO_EEPROM || p_r->buf[2] == YIT_RST_DLL_FATAL_ERROR) {
				inval_yit_cmd(p_r);
				return ((gfp_t) state_hw_reset);
			} else {
				inval_yit_cmd(p_r);
				return ((gfp_t) state_config_op_band);
			}
			break;
		case YIT_REMOTE_PARAMETERS_CHANGED :
			if (p_r->buf[0] != YIT_INDICATION) {
				msg(INF, "IT7# REMOTE_PARAMETERS_CHANGED error\n");
				inval_yit_cmd(p_r);
				return ((gfp_t) state_soft_reset);
			}
#if YIT_LOG_COMMANDS == 1
			log_rcv_cmd(p_r);
#endif
			inval_yit_cmd(p_r);
			break;
		default :
#if YIT_LOG_COMMANDS == 1
			msg(INF, "IT7# unexpected %s %s in TX_RESP1 state\n",
			    cmd_type_arr[p_r->buf[0] & 0x07],
			    find_txt_item(p_r->buf[1], cmd_dsc_ary, "IDX_ERR"));
#else
			msg(INF, "IT7# unexpected %s %X in TX_RESP1 state\n",
			    cmd_type_arr[p_r->buf[0] & 0x07], p_r->buf[1]);
#endif
			inval_yit_cmd(p_r);
			return ((gfp_t) state_soft_reset);
		}
	}
}

/**
 * rx_packet
 */
static gfp_t rx_packet(struct yit_cmd *p_r)
{
	if (p_r->buf[0] != YIT_INDICATION) {
		msg(INF, "IT7# RX_PACKET error\n");
		return ((gfp_t) state_soft_reset);
	}
#if YIT_LOG_COMMANDS == 1
	log_rx_pkt(p_r);
#endif
	if (p_r->buf[6] > 7 && p_r->buf[6] < 12) {
		if (rx_intra_clbk) {
			if ((adm_tag = (*rx_intra_clbk)(p_r))) {
				return ((gfp_t) state_tx_resp1);
			}
		}
	} else {
		if (rx_inter_clbk) {
			if ((adm_tag = (*rx_inter_clbk)(p_r))) {
				return ((gfp_t) state_tx_resp1);
			}
		}
	}
	return (NULL);
}

/**
 * yit_lv_net
 */
static void yit_lv_net(void)
{
	yit_snd_buf[0] = 0xCA;
	yit_snd_buf[1] = 0x02;
	yit_snd_buf[2] = 0x00;
	yit_snd_buf[3] = YIT_REQUEST;
	yit_snd_buf[4] = YIT_LEAVE_NETWORK;
	yit_sum_snd_buf();
#if YIT_LOG_COMMANDS == 1
	yit_log_snd_cmd();
#endif
	ser_dev_snd(ser_dev, yit_snd_buf, 6);
        noop_cnt = 0;
}

/**
 * valid_resp
 */
static boolean_t valid_resp(enum yit_opc opc, struct yit_cmd *p_r)
{
	if (p_r->buf[0] != YIT_RESPONSE || p_r->buf[1] != opc) {
#if YIT_LOG_COMMANDS == 1
		msg(INF, "IT7# %s error\n", find_txt_item(opc, cmd_dsc_ary, "IDX_ERR"));
#else
		msg(INF, "IT7# command %X error\n", opc);
#endif
		return (FALSE);
	}
	if (p_r->buf[2] != 0x01) {
#if YIT_LOG_COMMANDS == 1
		msg(INF, "IT7# %s failed\n", find_txt_item(opc, cmd_dsc_ary, "IDX_ERR"));
#else
		msg(INF, "IT7# command %X failed\n", opc);
#endif
		return (FALSE);
	}
	return (TRUE);
}

/**
 * inval_rxp_dfr
 */
static void inval_rxp_dfr(void)
{
	if (rxp_dfr) {
		inval_yit_cmd(rxp_dfr);
		rxp_dfr = NULL;
	}
}

/**
 * yit_sum_snd_buf
 */
void yit_sum_snd_buf(void)
{
	uint8_t s;
	int sz, i;

	s = 0;
	if ((sz = fr_lit_en(yit_snd_buf + 1) + 2) + 2 > YIT_SND_BUF_SIZE) {
		crit_err_exit(UNEXP_PROG_STATE);
	}
	for (i = 0; i < sz; i++) {
		s += *(yit_snd_buf + 1 + i);
	}
	*(yit_snd_buf + sz + 1) = s;
}

/**
 * to_lit_en
 */
static void to_lit_en(void *p, uint16_t n)
{
	*((uint8_t *) p) = n;
	*((uint8_t *) p + 1) = n >> 8;
}

/**
 * fr_lit_en
 */
static uint16_t fr_lit_en(void *p)
{
	return (*((uint8_t *) p) | (*((uint8_t *) p + 1) << 8));
}

#if YIT_LOG_COMMANDS == 1
static const char *intra_str = "INTRA";
static const char *inter_str = "INTER";
static const char *unicast_str = "UNICAST";
static const char *broadcast_str = "BROADCAST";

/**
 * log_rcv_cmd
 */
static void log_rcv_cmd(struct yit_cmd *p)
{
	const char *p_d;

	if (!p->valid || (*p->buf & 0x07) > 2) {
		crit_err_exit(UNEXP_PROG_STATE);
	}
	p_d = find_txt_item(*(p->buf + 1), cmd_dsc_ary, "IDX_ERR");
	msg(INF, "IT7# %s %s, size=%d\n", cmd_type_arr[*p->buf & 0x07], p_d, p->size);
}

/**
 * yit_log_snd_cmd
 */
void yit_log_snd_cmd(void)
{
	const char *p_d;

	p_d = find_txt_item(yit_snd_buf[4], cmd_dsc_ary, "IDX_ERR");
	if (p_d != tx_packet_str) {
		msg(INF, "IT7# %s %s, size=%d\n", cmd_type_arr[yit_snd_buf[3] & 0x07], p_d,
		    fr_lit_en(&yit_snd_buf[1]));
	} else {
		msg(INF, "IT7# <c TX_PACKET %d, size=%d\n", fr_lit_en(&yit_snd_buf[10]),
                    fr_lit_en(&yit_snd_buf[1]));
	}
}

/**
 * log_rx_pkt
 */
static void log_rx_pkt(struct yit_cmd *p)
{
	const char *p_n;
	const char *p_t;

	if (p->buf[6] > 7 && p->buf[6] < 12) {
		p_n = intra_str;
	} else {
		p_n = inter_str;
	}
	if (p->buf[3]) {
		p_t = broadcast_str;
	} else {
		p_t = unicast_str;
	}
	msg(INF, "IT7# i> RX_PACKET %s,%s dest=%s size=%d\n", p_n, p_t,
	    find_txt_item(p->buf[2], rx_pkt_type_dsc_ary, "IDX_ERR"), p->size);
}
#endif

#if TERMOUT == 1
/**
 * log_sn
 */
static void log_sn(void)
{
	char bf[33];

	for (int i = 0, j = 15; i < 16; i++, j--) {
		sprintf(bf + i * 2, "%.2X", sn[j]);
	}
	msg(INF, "IT7# s/n: %s\n", bf);
}

/**
 * log_fw_ver
 */
static void log_fw_ver(void)
{
	msg(INF, "IT7# fwv: %d.%d.%d\n", fw_ver.maj, fw_ver.min, fw_ver.bld);
}
#endif
