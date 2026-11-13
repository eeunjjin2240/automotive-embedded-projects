/*
 * @file    ds_twr_responder_int.c
 * @brief   DS-TWR responder using interrupts
 *
 * This example converts ds_twr_responder.c to use DW IC interrupts
 * for event handling instead of polling.
 */

#include "deca_probe_interface.h"
#include <config_options.h>
#include <deca_device_api.h>
#include <deca_spi.h>
#include <example_selection.h>
#include <port.h>
#include <shared_defines.h>
#include <shared_functions.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <dw3000_deca_regs.h>

#if defined(TEST_DS_TWR_RESPONDER_irq)

extern void test_run_info(unsigned char *data);

#define APP_NAME            "DS TWR RESP IRQ v1.0"
#define ANCHOR 4

/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
	5,                /* Channel number. */
	DWT_PLEN_4096,     /* Preamble length. Used in TX only. */
	DWT_PAC32,         /* Preamble acquisition chunk size. Used in RX only. */
	9,                /* TX preamble code. Used in TX only. */
	9,                /* RX preamble code. Used in RX only. */
	2,                /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
	DWT_BR_850K,       /* Data rate. */
	DWT_PHRMODE_STD,  /* PHY header mode. */
	DWT_PHRRATE_STD,  /* PHY header rate. */
	(4096 + 1 + 16 - 32),    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
	DWT_STS_MODE_OFF, /* STS disabled */
	DWT_STS_LEN_64,   /* STS length see allowed values in Enum dwt_sts_lengths_e */
	DWT_PDOA_M0       /* PDOA mode off */
};


/* Index to access the sequence number of the blink frame in the tx_msg array. */
#define BLINK_FRAME_SN_IDX 1

// ds_twr_responder macro

/* Default antenna delay values for 64 MHz PRF. See NOTE 1 below. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* Length of the common part of the message (up to and including the function code, see NOTE 2 below). */
#define ALL_MSG_COMMON_LEN 10

/* Index to access some of the fields in the frames involved in the process. */
#define ALL_MSG_SN_IDX            2
#define FINAL_MSG_POLL_TX_TS_IDX  10
#define FINAL_MSG_RESP_RX_TS_IDX  15
#define FINAL_MSG_FINAL_TX_TS_IDX 20

/* Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 24

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW IC's delayed TX function. This includes the
 * frame length of approximately 190 us with above configuration. */
#define FRAME_LENGTH 200
#define ANCHOR_WAIT 4000
#define POLL_RX_TO_RESP_TX_DLY_UUS (8000 + FRAME_LENGTH)  // 500us 정도인데 충분히 여유있게 설정, 8000
#define FINAL_RX_TO_TS_TX_DLY_UUS (8000 + FRAME_LENGTH)
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW IC's wait for response feature. */
//#define RESP_TX_TO_FINAL_RX_DLY_UUS 0
#define RESP_TX_TO_FINAL_RX_DLY_UUS 3000	// 전력 아끼는설정

/* Receive final timeout. See NOTE 5 below. */
#define FINAL_RX_TIMEOUT_UUS 30000

/* Preamble timeout, in multiple of PAC size. See NOTE 6 below. */
#define PRE_TIMEOUT 128

/* Buffer to store received frame. See NOTE 5 below. */

/* Declaration of static functions. */
static void rx_ok_cb(const dwt_cb_data_t *cb_data);
static void rx_to_cb(const dwt_cb_data_t *cb_data);
static void rx_err_cb(const dwt_cb_data_t *cb_data);
static void tx_conf_cb(const dwt_cb_data_t *cb_data);

/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
 * temperature. These values can be calibrated prior to taking reference measurements. See NOTE 6 below. */
extern dwt_txconfig_t txconfig_options;

// ds_twr_responder variables

/* Frames used in the ranging process. See NOTE 2 below. */
static uint8_t rx_poll_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 0x01, 0x00, 'V', 'E', 0x21 };
static uint8_t tx_resp1_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 0x01, 0x00, 'W', 'A', 0x10, 0x02, 0, 0 };
static uint8_t tx_resp2_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 0x02, 0x00, 'W', 'A', 0x10, 0x02, 0, 0 };
static uint8_t tx_resp3_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 0x03, 0x00, 'W', 'A', 0x10, 0x02, 0, 0 };
static uint8_t tx_resp4_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 0x04, 0x00, 'W', 'A', 0x10, 0x02, 0, 0 };
static uint8_t rx_final_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 0x01, 0x00, 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
static uint8_t tx_final_ts1_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 0x01, 0x00, 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
static uint8_t tx_final_ts2_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 0x02, 0x00, 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
static uint8_t tx_final_ts3_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 0x03, 0x00, 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
static uint8_t tx_final_ts4_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 0x04, 0x00, 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;

static uint8_t rx_buffer[RX_BUF_LEN];

/* Timestamps of frames transmission/reception. */
static uint64_t poll_rx_ts[4] = {0, 0, 0, 0};
static uint64_t resp_tx_ts[4] = {0, 0, 0, 0};
static uint64_t final_rx_ts[4] = {0, 0, 0, 0};

/* State machine */
typedef enum {
	S_POLL_WAIT,
	S_RESP_WAIT,
	S_FINAL_WAIT
} state_e;

static volatile state_e state = S_POLL_WAIT;

static uint16_t frame_len;
static uint32_t resp_tx_time;
static uint32_t final_ts_tx_time;

static int ret;

/* Function prototype (place this in your header or at the top of the C file) */
static void uwb_enable_receive(void);

//코드 실행시간 계산용, 딜레이 설정시 참고
static uint32_t cb_t1;
static uint32_t cb_t2;
static uint32_t cb_t3;
static uint32_t cb_t4;
static uint32_t cb_t10;
static uint32_t cb_t11;
static double poll_rx2res_tx_processing;
static double res_tx2final_rx_delay;
static double res_tx2final_rx_processing;
static char delay_info_str[128];

static char info_str[128];

/**
 * Application entry point.
 */
int ds_twr_responder_irq(void)
{
    dwt_callbacks_s cbs = {NULL};

    /* Display application name on LCD. */
    test_run_info((unsigned char *)APP_NAME);

    /* Configure SPI rate, DW3000 supports up to 38 MHz */
    port_set_dw_ic_spi_fastrate();

    /* Reset DW IC */
    reset_DWIC(); /* Target specific drive of RSTn line into DW IC low for a period. */

    Sleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

    /* Probe for the correct device driver. */
    dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf);

    while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */ { };

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        test_run_info((unsigned char *)"INIT FAILED     ");
        while (1) { };
    }

    /* Configure DW IC. See NOTE 2 below. */
    /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
    if (dwt_configure(&config))
    {
        test_run_info((unsigned char *)"CONFIG FAILED     ");
        while (1) { };
    }

    /* Configure the TX spectrum parameters (power, PG delay and PG count) */
    dwt_configuretxrf(&txconfig_options);

    /* Define all the callback functions that will be called by the DW IC driver as a result of DW IC events. */
    cbs.cbTxDone = tx_conf_cb;
    cbs.cbRxOk = rx_ok_cb;
    cbs.cbRxTo = rx_to_cb;
    cbs.cbRxErr = rx_err_cb;

    /* Register the call-backs (SPI CRC error callback is not used). */
    dwt_setcallbacks(&cbs);

    /* Enable wanted interrupts (TX confirmation, RX good frames, RX timeouts and RX errors). */
    dwt_setinterrupt(DWT_INT_TXFRS_BIT_MASK | DWT_INT_RXFCG_BIT_MASK | DWT_INT_RXFTO_BIT_MASK | DWT_INT_RXPTO_BIT_MASK | DWT_INT_RXPHE_BIT_MASK |
    				 DWT_INT_RXFCE_BIT_MASK | DWT_INT_RXFSL_BIT_MASK | DWT_INT_RXSTO_BIT_MASK | DWT_INT_RXOVRR_BIT_MASK,
					 0, DWT_ENABLE_INT);

    /* Install DW IC IRQ handler. */
    port_set_dwic_isr(dwt_isr);

    /*Clearing the SPI ready interrupt*/
	dwt_writesysstatuslo(DWT_INT_RCINIT_BIT_MASK | DWT_INT_SPIRDY_BIT_MASK);

	// 인터럽트 clear
	dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

    /* Antenna delays */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);


    /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug, and also TX/RX LEDs
	 * Note, in real low power applications the LEDs should not be used. */
	dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
	// dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

    // tx power 설정
    dwt_settxpower(0xfdfdfdfd);

    uwb_enable_receive();

    /* Loop forever sending and receiving frames periodically. */
    while (1)
    {

    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn rx_ok_cb()
 *
 * @brief Callback to process RX good frame events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
static void rx_ok_cb(const dwt_cb_data_t *cb_data)
{
	(void)cb_data;
	uint8_t anchor_id;

    // 인터럽트 비트 클리어
	dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD);

	if (state == S_POLL_WAIT) {

		cb_t1 = dwt_readsystimestamphi32();

	    /* A frame has been received, read it into the local buffer. */
		frame_len = dwt_getframelength(0);
		if (frame_len <= RX_BUF_LEN)
		{
			dwt_readrxdata(rx_buffer, frame_len, 0);
		}

		/* Poll received? */
		rx_buffer[ALL_MSG_SN_IDX] = 0;
		if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)
		{
			//test_run_info("ppp");
			if(ANCHOR == 1) {
				/* Retrieve poll reception timestamp. */
				poll_rx_ts[0] = get_rx_timestamp_u64();
				/* Set send time for response. See NOTE 9 below. */
				resp_tx_time = (poll_rx_ts[0] + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
				dwt_setdelayedtrxtime(resp_tx_time);

				/* Set expected delay and timeout for final message reception. See NOTE 4 and 5 below. */
				dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
				dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);
				/* Set preamble timeout for expected frames. See NOTE 6 below. */
				dwt_setpreambledetecttimeout(0);
				/* Write and send the response message. See NOTE 10 below.*/
				tx_resp1_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
				dwt_writetxdata(sizeof(tx_resp1_msg), tx_resp1_msg, 0); /* Zero offset in TX buffer. */
				dwt_writetxfctrl(sizeof(tx_resp1_msg) + FCS_LEN, 0, 1);
			}
			if(ANCHOR == 2) {
				/* Retrieve poll reception timestamp. */
				poll_rx_ts[1] = get_rx_timestamp_u64();
				/* Set send time for response. See NOTE 9 below. */
				resp_tx_time = (poll_rx_ts[1] + ((POLL_RX_TO_RESP_TX_DLY_UUS + ANCHOR_WAIT) * UUS_TO_DWT_TIME)) >> 8;
				dwt_setdelayedtrxtime(resp_tx_time);

				/* Set expected delay and timeout for final message reception. See NOTE 4 and 5 below. */
				dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
				dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);
				/* Set preamble timeout for expected frames. See NOTE 6 below. */
				dwt_setpreambledetecttimeout(0);
				/* Write and send the response message. See NOTE 10 below.*/
				tx_resp2_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
				dwt_writetxdata(sizeof(tx_resp2_msg), tx_resp2_msg, 0); /* Zero offset in TX buffer. */
				dwt_writetxfctrl(sizeof(tx_resp2_msg) + FCS_LEN, 0, 1);
			}
			if(ANCHOR == 3) {
				/* Retrieve poll reception timestamp. */
				poll_rx_ts[2] = get_rx_timestamp_u64();
				/* Set send time for response. See NOTE 9 below. */
				resp_tx_time = (poll_rx_ts[2] + ((POLL_RX_TO_RESP_TX_DLY_UUS + 2 * ANCHOR_WAIT) * UUS_TO_DWT_TIME)) >> 8;
				dwt_setdelayedtrxtime(resp_tx_time);

				/* Set expected delay and timeout for final message reception. See NOTE 4 and 5 below. */
				dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
				dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);
				/* Set preamble timeout for expected frames. See NOTE 6 below. */
				dwt_setpreambledetecttimeout(0);
				/* Write and send the response message. See NOTE 10 below.*/
				tx_resp3_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
				dwt_writetxdata(sizeof(tx_resp3_msg), tx_resp3_msg, 0); /* Zero offset in TX buffer. */
				dwt_writetxfctrl(sizeof(tx_resp3_msg) + FCS_LEN, 0, 1);
			}
			else if(ANCHOR == 4) {
				/* Retrieve poll reception timestamp. */
				poll_rx_ts[3] = get_rx_timestamp_u64();
				/* Set send time for response. See NOTE 9 below. */
				resp_tx_time = (poll_rx_ts[3] + ((POLL_RX_TO_RESP_TX_DLY_UUS + 3 * ANCHOR_WAIT) * UUS_TO_DWT_TIME)) >> 8;
				dwt_setdelayedtrxtime(resp_tx_time);

				/* Set expected delay and timeout for final message reception. See NOTE 4 and 5 below. */
				dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
				dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);
				/* Set preamble timeout for expected frames. See NOTE 6 below. */
				dwt_setpreambledetecttimeout(0);
				/* Write and send the response message. See NOTE 10 below.*/
				tx_resp4_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
				dwt_writetxdata(sizeof(tx_resp4_msg), tx_resp4_msg, 0); /* Zero offset in TX buffer. */
				dwt_writetxfctrl(sizeof(tx_resp4_msg) + FCS_LEN, 0, 1);
			}

			ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
			/* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 11 below. */
			if (ret != DWT_SUCCESS)
			{
				uwb_enable_receive();
			}
			else{
				state = S_FINAL_WAIT;
			}

		}
		else {
			uwb_enable_receive();
		}
		cb_t2 = dwt_readsystimestamphi32();
	}
	else if (state == S_FINAL_WAIT) {

		cb_t4 = dwt_readsystimestamphi32();

	    /* A frame has been received, read it into the local buffer. */
		frame_len = dwt_getframelength(0);
		if (frame_len <= RX_BUF_LEN)
		{
			dwt_readrxdata(rx_buffer, frame_len, 0);
		}

		rx_buffer[ALL_MSG_SN_IDX] = 0;

		if (memcmp(rx_buffer, rx_final_msg, ALL_MSG_COMMON_LEN) == 0)
		{
			uint32_t poll_tx_ts, resp_rx_ts, final_tx_ts;
			if(ANCHOR == 1) {
				/* Retrieve response transmission and final reception timestamps. */
				resp_tx_ts[0] = get_tx_timestamp_u64();
				final_rx_ts[0] = get_rx_timestamp_u64();
				/* Set send time for response. See NOTE 9 below. */
				final_ts_tx_time = (final_rx_ts[0] + (FINAL_RX_TO_TS_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
				dwt_setdelayedtrxtime(final_ts_tx_time);
				// timestamp를 final_ts msg에 담기
			    final_msg_set_ts(&tx_final_ts1_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_rx_ts[0]);
			    final_msg_set_ts(&tx_final_ts1_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_tx_ts[0]);
			    final_msg_set_ts(&tx_final_ts1_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_rx_ts[0]);
				tx_final_ts1_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
				dwt_writetxdata(sizeof(tx_final_ts1_msg), tx_final_ts1_msg, 0); /* Zero offset in TX buffer. */
				dwt_writetxfctrl(sizeof(tx_final_ts1_msg) + FCS_LEN, 0, 1);
			}
			else if(ANCHOR == 2) {
				/* Retrieve response transmission and final reception timestamps. */
				resp_tx_ts[1] = get_tx_timestamp_u64();
				final_rx_ts[1] = get_rx_timestamp_u64();
				/* Set send time for response. See NOTE 9 below. */
				final_ts_tx_time = (final_rx_ts[1] + ((FINAL_RX_TO_TS_TX_DLY_UUS + ANCHOR_WAIT + 2000) * UUS_TO_DWT_TIME)) >> 8;
				dwt_setdelayedtrxtime(final_ts_tx_time);
				// timestamp를 final_ts msg에 담기
			    final_msg_set_ts(&tx_final_ts2_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_rx_ts[1]);
			    final_msg_set_ts(&tx_final_ts2_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_tx_ts[1]);
			    final_msg_set_ts(&tx_final_ts2_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_rx_ts[1]);
				tx_final_ts2_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
				dwt_writetxdata(sizeof(tx_final_ts2_msg), tx_final_ts2_msg, 0); /* Zero offset in TX buffer. */
				dwt_writetxfctrl(sizeof(tx_final_ts2_msg) + FCS_LEN, 0, 1);
			}
			else if(ANCHOR == 3) {
				/* Retrieve response transmission and final reception timestamps. */
				resp_tx_ts[2] = get_tx_timestamp_u64();
				final_rx_ts[2] = get_rx_timestamp_u64();
				/* Set send time for response. See NOTE 9 below. */
				final_ts_tx_time = (final_rx_ts[2] + ((FINAL_RX_TO_TS_TX_DLY_UUS + 2 * (ANCHOR_WAIT + 2000)) * UUS_TO_DWT_TIME)) >> 8;
				dwt_setdelayedtrxtime(final_ts_tx_time);
				// timestamp를 final_ts msg에 담기
			    final_msg_set_ts(&tx_final_ts3_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_rx_ts[2]);
			    final_msg_set_ts(&tx_final_ts3_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_tx_ts[2]);
			    final_msg_set_ts(&tx_final_ts3_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_rx_ts[2]);
				tx_final_ts3_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
				dwt_writetxdata(sizeof(tx_final_ts3_msg), tx_final_ts3_msg, 0); /* Zero offset in TX buffer. */
				dwt_writetxfctrl(sizeof(tx_final_ts3_msg) + FCS_LEN, 0, 1);
			}
			else if(ANCHOR == 4) {
				/* Retrieve response transmission and final reception timestamps. */
				resp_tx_ts[3] = get_tx_timestamp_u64();
				final_rx_ts[3] = get_rx_timestamp_u64();
				/* Set send time for response. See NOTE 9 below. */
				final_ts_tx_time = (final_rx_ts[3] + ((FINAL_RX_TO_TS_TX_DLY_UUS + 3 * (ANCHOR_WAIT+2000)) * UUS_TO_DWT_TIME)) >> 8;
				dwt_setdelayedtrxtime(final_ts_tx_time);
				//timestamp를 final_ts msg에 담기
			    final_msg_set_ts(&tx_final_ts4_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_rx_ts[3]);
			    final_msg_set_ts(&tx_final_ts4_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_tx_ts[3]);
			    final_msg_set_ts(&tx_final_ts4_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_rx_ts[3]);
				tx_final_ts4_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
				dwt_writetxdata(sizeof(tx_final_ts4_msg), tx_final_ts4_msg, 0); /* Zero offset in TX buffer. */
				dwt_writetxfctrl(sizeof(tx_final_ts4_msg) + FCS_LEN, 0, 1);
			}
			ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
		    //tx 실패할 때 확인할 필요 있음
			if (ret != DWT_SUCCESS)
			{
				uwb_enable_receive();
			}
			uwb_enable_receive();
		}
		else {
			dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);
	        dwt_rxenable(DWT_START_RX_IMMEDIATE);
		}

	}
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn rx_to_cb()
 *
 * @brief Callback to process RX timeout events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
static void rx_to_cb(const dwt_cb_data_t *cb_data)
{
    (void)cb_data;

    /* RX timeout: 클리어 후 백오프 */
	dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

	//test_run_info((unsigned char *) "flag to");
	uwb_enable_receive();

}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn rx_err_cb()
 *
 * @brief Callback to process RX error events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
static void rx_err_cb(const dwt_cb_data_t *cb_data)
{
    (void)cb_data;

    /* RX 에러(프레임 손상 등): 관련 에러 비트 클리어 */
    dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

    //test_run_info((unsigned char *) "flag err");
    uwb_enable_receive();

}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn tx_conf_cb()
 *
 * @brief Callback to process TX confirmation events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
static void tx_conf_cb(const dwt_cb_data_t *cb_data)
{
	cb_t3 = dwt_readsystimestamphi32();
	/*if (state == S_FINAL_WAIT) {
		res_tx2final_rx_processing = (double)(cb_t3 - cb_t1)*0.008;
		sprintf(delay_info_str, "res tx 2 final rx delay: %.3f us", res_tx2final_rx_processing);
		test_run_info((unsigned char *)delay_info_str);
	}
	else if(state == S_POLL_WAIT) {
		res_tx2final_rx_processing = (double)(cb_t3 - cb_t4)*0.008;
		sprintf(delay_info_str, "res tx 2 final rx delay: %.3f us", res_tx2final_rx_processing);
		test_run_info((unsigned char *)delay_info_str);
	}*/

    (void)cb_data;
    /* This callback has been defined so that a breakpoint can be put here to check it is correctly called but there is actually nothing specific to
     * do on transmission confirmation in this example. Typically, we could activate reception for the response here but this is automatically handled
     * by DW IC using DWT_RESPONSE_EXPECTED parameter when calling dwt_starttx().
     * An actual application that would not need this callback could simply not define it and set the corresponding field to NULL when calling
     * dwt_setcallbacks(). The ISR will not call it which will allow to save some interrupt processing time. */

    // Poll 전송 완료 인터럽트 비트 클리어
	dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
	//test_run_info((unsigned char *) "flag tx_conf");

}

static void uwb_enable_receive(void)
{
    /* Set preamble detect timeout (in PAC size units) */
    dwt_setpreambledetecttimeout(PRE_TIMEOUT);

    /* Clear any lingering RX timeout to restart ranging */
    dwt_setrxtimeout(0);

    /* Enter RX immediately */
    ret = dwt_rxenable(DWT_START_RX_IMMEDIATE);

    /*if (ret == DWT_SUCCESS) {
        test_run_info((unsigned char *)"RX_ENABLE: OK   ");
    }
    else {
        test_run_info((unsigned char *)"RX_ENABLE: ERR! ");
    }*/

    state = S_POLL_WAIT;
}

#endif
/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The device ID is a hard coded constant in the blink to keep the example simple but for a real product every device should have a unique ID.
 *    For development purposes it is possible to generate a DW IC unique ID by combining the Lot ID & Part Number values programmed into the
 *    DW IC during its manufacture. However there is no guarantee this will not conflict with someone else's implementation. We recommended that
 *    customers buy a block of addresses from the IEEE Registration Authority for their production items. See "EUI" in the DW IC User Manual.
 * 2. Desired configuration by user may be different to the current programmed configuration. dwt_configure is called to set desired
 *    configuration.
 * 3. TX to RX delay can be set to 0 to activate reception immediately after transmission. But, on the responder side, it takes time to process the
 *    received frame and generate the response (this has been measured experimentally to be around 70 us). Using an RX to TX delay slightly less than
 *    this minimum turn-around time allows the application to make the communication efficient while reducing power consumption by adjusting the time
 *    spent with the receiver activated.
 * 4. This timeout is for complete reception of a frame, i.e. timeout duration must take into account the length of the expected frame. Here the value
 *    is arbitrary but chosen large enough to make sure that there is enough time to receive a complete frame sent by the "RX then send a response"
 *    example at the 110k data rate used (around 3 ms).
 * 5. In this example, maximum frame length is set to 127 bytes which is 802.15.4 UWB standard maximum frame length. DW IC supports an extended frame
 *    length (up to 1023 bytes long) mode which is not used in this example.
 * 6. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW IC OTP memory.
 * 7. dwt_writetxdata() takes the full size of tx_msg as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW IC. This means that our tx_msg could be two bytes shorter without losing any data (but the sizeof would not
 *    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
 ****************************************************************************************************************************************************/
