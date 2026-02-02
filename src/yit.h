/*
 * yit.h
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

#ifndef YIT_H
#define YIT_H

enum yit_opc {
	YIT_NO_OPERATION = 0x00,
	YIT_GET_VERSION = 0x01,
	YIT_GET_FREE_MEMORY = 0x02,
	YIT_READ_FROM_NVM = 0x05,
	YIT_WRITE_TO_NVM = 0x06,
	YIT_RESET = 0x20,
	YIT_GO_ONLINE = 0x22,
	YIT_GO_OFFLINE = 0x23,
	YIT_SET_PREDEFINED_PARAMETERS = 0x40,
	YIT_SET_DEVICE_PARAMETERS = 0x41,
	YIT_GET_DEVICE_PARAMETERS = 0x42,
	YIT_SAVE_DEVICE_PARAMETERS = 0x43,
	YIT_REMOTE_PARAMETERS_CHANGED = 0x4C,
	YIT_TX_PACKET = 0x60,
	YIT_GET_NC_DATABASE_SIZE = 0x65,
	YIT_RX_PACKET = 0x68,
	YIT_GET_NODE_INFORMATION = 0x69,
	YIT_DELETE_NODE_INFORMATION = 0x6A,
	YIT_ADMISSION_APPROVAL_RESP_FROM_APP = 0xA4,
	YIT_LEAVE_NETWORK = 0xA6,
	YIT_CONNECTIVITY_STATUS_WITH_RS = 0xB1,
	YIT_NODE_LEFT_NETWORK = 0xB3,
	YIT_GET_ADMISSION_APPROVAL_FROM_APP = 0xB8,
	YIT_ADMISSION_REFUSE = 0xB9,
	YIT_CONNECTED_TO_NC = 0xBA,
	YIT_DISCONNECTED_FROM_NC = 0xBB,
	YIT_NEW_CONNECTION_TO_NC = 0xBE,
	YIT_NETWORK_ID_ASSIGNED = 0xBF
};

enum yit_cmd_type {
	YIT_REQUEST,
	YIT_RESPONSE,
	YIT_INDICATION
};

enum yit_op_band {
	YIT_OP_BAND_FCC  = 0,
	YIT_OP_BAND_CA   = 2,
	YIT_OP_BAND_CB   = 3,
	YIT_OP_BAND_CA3  = 4,
	YIT_OP_BAND_ARIB = 128
};

enum yit_region {
	YIT_REGION_FCC,
	YIT_REGION_ARIB,
	YIT_REGION_CA,
	YIT_REGION_CB,
	YIT_REGION_CA3,
	YIT_REGION_DFLT = 0xFF
};

enum yit_data_serv_type {
	YIT_DATA_SERV_INTRA_BROADCAST,
	YIT_DATA_SERV_INTRA_UNICAST,
	YIT_DATA_SERV_INTRA_UNICAST_SN,
	YIT_DATA_SERV_INTER_BROADCAST,
	YIT_DATA_SERV_INTER_UNICAST
};

enum yit_pkt_prio {
	YIT_PKT_PRIO_NORMAL,
	YIT_PKT_PRIO_HIGH,
	YIT_PKT_PRIO_EMERGENCY
};

enum yit_pkt_ack_serv {
	YIT_PKT_SERV_NO_ACK,
	YIT_PKT_SERV_ACK_REQ
};

enum yit_tx_gain {
	YIT_TX_GAIN_0,
	YIT_TX_GAIN_1,
        YIT_TX_GAIN_2,
        YIT_TX_GAIN_3,
        YIT_TX_GAIN_4,
        YIT_TX_GAIN_5,
	YIT_TX_GAIN_6,
        YIT_TX_GAIN_7
};

enum yit_tx_encrypted {
	YIT_TX_ENCRYPTED_0,
	YIT_TX_ENCRYPTED_1
};

enum yit_rx_pkt_type {
	YIT_PKT_TYPE_TO_DEV_IN_OTHER_NET,
        YIT_PKT_TYPE_TO_DEV_IN_MY_NET,
        YIT_PKT_TYPE_TO_ME,
        YIT_PKT_TYPE_RETRANSMITTED,
        YIT_PKT_TYPE_SPOOFED,
	YIT_PKT_TYPE_MALFORMED
};

enum yit_led_ctl {
	YIT_LED_OFF,
	YIT_LED_ON,
	YIT_LED_BLINK_NORMAL,
	YIT_LED_BLINK_FAST
};

#define YIT_RXP_ADDR_TYPE 17
#define YIT_RXP_SERV_TYPE 3

struct yit_stats {
	int disc_nc;
	int hw_rst;
	int sw_rst;
	int wd_rst;
        int txp_adm_err;
	int txp_trn_err;
};

struct yit_it700_fw_ver {
	int maj;
	int min;
	int bld;
};

extern uint8_t yit_snd_buf[];

/**
 * yit_sum_snd_buf
 */
void yit_sum_snd_buf(void);

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
		void (*inval_yit_cmd_fn)(struct yit_cmd *p_r));

/**
 * yit_stats
 */
struct yit_stats *yit_stats(void);

/**
 * yit_it700_fw_ver
 */
struct yit_it700_fw_ver *yit_it700_fw_ver(void);

/**
 * yit_force_leave_net
 */
void yit_force_leave_net(void);

#if YIT_LOG_COMMANDS == 1
/**
 * yit_log_snd_cmd
 */
void yit_log_snd_cmd(void);
#endif

#endif
