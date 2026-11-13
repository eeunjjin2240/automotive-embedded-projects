/*
 * @file    ds_twr_initiator_int.c
 * @brief   DS-TWR initiator using interrupts
 *
 * This example converts ds_twr_initiator.c to use DW IC interrupts
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
#include <stdlib.h>
#include <stdint.h>
#include <stm32f4xx_hal.h>
#include <ESP8266_HAL.h>
#include <main.h>
#include <usart.h>
#include <math.h>

#if defined(TEST_DS_TWR_INITIATOR_irq)

extern void test_run_info(unsigned char *data);

#define APP_NAME            "DS TWR INIT IRQ v1.0"

static uint8_t num_anchors = 4;  // 앵커 개수
static uint8_t flag_resp_ts = 0;
static uint8_t flag_resp = 0;
static uint8_t resp_anchor[4] = {0, 0, 0, 0};

uint32_t start_1, end_1, start_2, end_2;
float duration_us_1, duration_us_2;
int num_iter;


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

/* Current inter-frame delay period.
 * This global static variable is also used as the mechanism to signal events to the background main loop from the interrupt handler callbacks,
 * which set it to positive delay values. */

//ds_twr_initiator macros

/* Default antenna delay values for 64 MHz PRF. See NOTE 1 below. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define ANCHOR_WAIT 6000

/* Frames used in the ranging process. See NOTE 2 below. */
static uint8_t tx_poll_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 0x01, 0x00, 'V', 'E', 0x21 };
static uint8_t rx_resp1_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 0x01, 0x00, 'W', 'A', 0x10, 0x02, 0, 0 };
static uint8_t rx_resp2_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 0x02, 0x00, 'W', 'A', 0x10, 0x02, 0, 0 };
static uint8_t rx_resp3_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 0x03, 0x00, 'W', 'A', 0x10, 0x02, 0, 0 };
static uint8_t rx_resp4_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 0x04, 0x00, 'W', 'A', 0x10, 0x02, 0, 0 };
static uint8_t tx_final_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 0x01, 0x00, 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

/* Length of the common part of the message (up to and including the function code, see NOTE 2 below). */
#define ALL_MSG_COMMON_LEN 10

/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX            2
#define FINAL_MSG_POLL_TX_TS_IDX  10
#define FINAL_MSG_RESP_RX_TS_IDX  15
#define FINAL_MSG_FINAL_TX_TS_IDX 20

/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 30  // resp msg와 final_ts msg 길이 이상으로 설정

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW IC's wait for response feature. */
//#define POLL_TX_TO_RESP_RX_DLY_UUS 100	// 거의 0임
#define POLL_TX_TO_RESP_RX_DLY_UUS 3000	// init processing 으로 딜레이 생기므로 전력 아낌
#define FINAL_TX_TO_TS_RX_DLY_UUS 3000

/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW IC's delayed TX function.
 * This value is required to be larger than POLL_TX_TO_RESP_RX_DLY_UUS. Please see NOTE 4 for more details. */
//#define RESP_RX_TO_FINAL_TX_DLY_UUS 1800 // 900us 정도인데 충분히 여유있게 설정
#define RESP_RX_TO_FINAL_TX_DLY_UUS 13000 // resp processing 타임 고려 딜레이 설정 7000
#define FINAL_TX_TO_POLL_TX_DLY_UUS 30000
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 20000
#define TS_RX_TIMEOUT_UUS 20000
/* Preamble timeout, in multiple of PAC size. See NOTE 7 below. */
#define PRE_TIMEOUT 0

/* Buffer to store received frame. See NOTE 5 below. */
static uint8_t rx_buffer[FRAME_LEN_MAX];

/* Declaration of static functions. */
static void rx_ok_cb(const dwt_cb_data_t *cb_data);
static void rx_to_cb(const dwt_cb_data_t *cb_data);
static void rx_err_cb(const dwt_cb_data_t *cb_data);
static void tx_conf_cb(const dwt_cb_data_t *cb_data);

/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
 * temperature. These values can be calibrated prior to taking reference measurements. See NOTE 6 below. */
extern dwt_txconfig_t txconfig_options;
// ds_twr_initiator variables

/* Time-stamps of frames transmission/reception, expressed in device time units. */
static uint64_t poll_tx_ts[4] = {0, 0, 0, 0};
static uint64_t resp_rx_ts[4] = {0, 0, 0, 0};
static uint64_t final_tx_ts[4] = {0, 0, 0, 0};
static uint64_t poll_rx_ts[4] = {0, 0, 0, 0};
static uint64_t resp_tx_ts[4] = {0, 0, 0, 0};
static uint64_t final_rx_ts[4] = {0, 0, 0, 0};
static double Ra[4] = {0, 0, 0, 0};
static double Rb[4] = {0, 0, 0, 0};
static double Da[4] = {0, 0, 0, 0};
static double Db[4] = {0, 0, 0, 0};
static int64_t tof_dtu[4] = {0, 0, 0, 0};
static double tof[4] = {0, 0, 0, 0};
double distances[4] = {0, 0, 0, 0};

static char info_str[128];

/* State machine for initiator. */
typedef enum {
    S_POLL_TX,
    S_RESP_RX,
    S_FINAL_TX
} initiator_state_e;

uint64_t latest_rx_time;

// 앵커의 위치
Point2D pos_anchors[4] = {
    {0.0f, 0.0f},
    {50.0f, 0.0f},
    {0.0f, 50.0f},
    {50.0f, 50.0f}
};

static volatile initiator_state_e state = S_POLL_TX;

/* Control flag from callbacks: sets inter-frame delay and wakes main loop. */
static volatile int32_t tx_delay_ms = -1;

static uint32_t current_time_high32;
static uint32_t delay_high32;
static uint32_t starttime;
static int ret;

/* Forward declarations */
static int send_poll(uint32_t poll_delay_uus);
static void send_final(void);

//코드 실행시간 계산용, 딜레이 설정시 참고
static uint32_t cb_t2;
static uint32_t cb_t3;

uint32_t last_poll_time = 0;
const uint32_t poll_interval_ms = 100; // 100ms 간격

// randon delay
static uint32_t get_random_delay(void);

static dwt_callbacks_s cbs = {NULL};

Point2D pos = {0.0f, 0.0f}; // 태그 좌표 초기값: (0,0)

//위치 추정 함수
void estimate_tag_position(Point2D *anchors, double *distances, uint8_t num_anchors, uint8_t flag_resp_ts, uint8_t * resp_anchor) {
    // 태그 위치 초기값: 직사각형 중간값
    pos.x = anchors[3].x / 2.0;
    pos.y = anchors[3].y / 2.0;
    // 삼각측량 변수
    Point2D A[2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};
    double dist1, dist2, d1, d2;
    int flag_A = 0;
    double N1 = 0.0f;
    double N2 = 0.0f;
    double B = 0.0f;
    // 최적화 알고리즘 변수
    double learning_rate;
    int max_iter = 5000;
    double epsilon = 1e-6f;

    // 삼각측량 알고리즘
    if(flag_resp_ts == 2) {
    	// 받은 2개의 앵커 좌표와 거리값 저장
    	for(int i = 0; i < num_anchors; i++) {
        	if(resp_anchor[i] == 0)  // 못받은 앵커는 무시
        		continue;
        	if((distances[i] <= 0) || (distances[i] > 1000))  // 쓰레기값이면 함수 종료
        		return;
        	if(flag_A == 0) {
        		A[0].x = anchors[i].x;
        		A[0].y = anchors[i].y;
        		dist1 = distances[i];
        		flag_A = 1;
        	}
        	else if(flag_A == 1) {
        		A[1].x = anchors[i].x;
        		A[1].y = anchors[i].y;
        		dist2 = distances[i];
        		flag_A = 0;
        	}
    	}

    	// case1, 3에 대한 삼각측량
    	if(A[0].y == A[1].y) {
    		N1 = (A[0].x < A[1].x) ? (A[1].x - A[0].x) : (A[0].x - A[1].x);
    		N2 = A[0].y;
    		d1 = (A[0].x < A[1].x) ? dist1 : dist2;
    		d2 = (A[0].x < A[1].x) ? dist2 : dist1;
    		pos.x = (d1*d1 - d2*d2 + N1*N1) / (2*N1);
    		B = sqrtf(d1*d1 - pos.x*pos.x);
    		pos.y = (A[0].y == 0) ? B : (N2-B);
    	}
    	// case2, 4에 대한 삼각측량
    	else if(A[0].x == A[1].x) {
    		N1 = A[0].x;
    		N2 = (A[0].y < A[1].y) ? (A[1].y - A[0].y) : (A[0].y - A[1].y);
    		d1 = (A[0].y < A[1].y) ? dist1 : dist2;
    		d2 = (A[0].y < A[1].y) ? dist2 : dist1;
    		pos.y = (d1*d1 - d2*d2 + N2*N2) / (2*N2);
    		B = sqrtf(d1*d1 - pos.y*pos.y);
    		pos.x = (A[0].x == 0) ? B : (N1-B);
    	}
    }

    // beacon-based location 최적화 알고리즘
    else if(flag_resp_ts > 2) {
        // x = x - learning_rate * grad   500번 반복
        for (int iter = 0; iter < max_iter; iter++) {
            double grad_x = 0.0f;
            double grad_y = 0.0f;
            double alpha_max = 0.00001;
            double k = 3.0;
            double f = 0.0f;

            // error function과 그 gradient 구하기 (앵커 개수만큼 더하기)
            for (int i = 0; i < num_anchors; i++) {
            	if(resp_anchor[i] == 0)  // 못받은 앵커 무시
            		continue;
            	if((distances[i] <= 0) || (distances[i] > 1000)) // 쓰레기 값이면 함수 종료
            		return;
                double dx = pos.x - anchors[i].x;
                double dy = pos.y - anchors[i].y;
                double dist_est = dx * dx + dy * dy - distances[i] * distances[i];
                double error = dist_est * dist_est;
                f += error;
                grad_x += 4.0f * dist_est * dx;
                grad_y += 4.0f * dist_est * dy;
            }
            // 업데이트
            learning_rate = alpha_max * sqrtf(f) / (sqrtf(f) + k);
            if (learning_rate < epsilon)
            	learning_rate = epsilon; // 최소 보장
            pos.x -= learning_rate * grad_x;
            pos.y -= learning_rate * grad_y;

            // 수렴 조건 -> 잘 안되면 없애고 해보기
            if (grad_x * grad_x + grad_y * grad_y < epsilon*epsilon)
            	break;
            num_iter = iter;
        }
        Server_Start();
        end_1 = DWT->CYCCNT;
        uint32_t cpu_clock_hz_1 = HAL_RCC_GetHCLKFreq(); // 또는 직접 클럭 값 입력
        duration_us_1 = ((float)(end_1 - start_1) / cpu_clock_hz_1) * 1e6f; // μs 단위로 변환
        char buf[64];
        sprintf(buf, "POS: x=%.2f, y=%.2f | iter=%d | alpha=%f | 실행 시간: %.2f us\r\n", pos.x, pos.y, num_iter, learning_rate, duration_us_1);
        test_run_info((unsigned char *)buf);
    }
}

int ds_twr_initiator_irq(void)
{

    /* Display application name on LCD. */
	test_run_info((unsigned char *)APP_NAME);

    srand(HAL_GetTick()); // 난수 시드 설정 (한 번만 실행)

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
    dwt_setinterrupt(DWT_INT_TXFRS_BIT_MASK | DWT_INT_RXFCG_BIT_MASK | DWT_INT_RXFTO_BIT_MASK | DWT_INT_RXPTO_BIT_MASK | DWT_INT_RXPHE_BIT_MASK
					 | DWT_INT_RXFCE_BIT_MASK | DWT_INT_RXFSL_BIT_MASK | DWT_INT_RXSTO_BIT_MASK,
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

    /* Ranging timing */
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    dwt_setpreambledetecttimeout(PRE_TIMEOUT);

    // tx power 설정
    dwt_settxpower(0xfdfdfdfd);

    do {
        send_poll(get_random_delay());
        last_poll_time = HAL_GetTick();
    } while ((ret != DWT_SUCCESS) || (HAL_GetTick() - last_poll_time > poll_interval_ms));

    state = S_POLL_TX;

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
	cb_t2 = dwt_readsystimestamphi32();

    (void)cb_data;
    uint16_t frame_len;
	uint8_t anchor_id;

    // 인터럽트 비트 클리어
    dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD);

    if (state == S_POLL_TX) {

    	/* A frame has been received, read it into the local buffer. */
    		frame_len = dwt_getframelength(0);
    		if (frame_len <= RX_BUF_LEN)
    		{
    			dwt_readrxdata(rx_buffer, frame_len, 0);
    		}

        /* Check that the frame is the expected response from the companion "DS TWR responder" example.
         * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
    		rx_buffer[ALL_MSG_SN_IDX] = 0;
    		anchor_id = rx_buffer[5];
    		//sprintf(info_str, "anchor ID: %d", anchor_id);
    		//test_run_info((unsigned char *)info_str);
			if(memcmp(rx_buffer, rx_resp1_msg, ALL_MSG_COMMON_LEN) == 0) {
				flag_resp += 1;
    			poll_tx_ts[0] = get_tx_timestamp_u64();
    			resp_rx_ts[0] = get_rx_timestamp_u64();
    			dwt_setrxtimeout(3 * ANCHOR_WAIT);
    			dwt_rxenable(DWT_START_RX_IMMEDIATE);
			}
			else if (memcmp(rx_buffer, rx_resp2_msg, ALL_MSG_COMMON_LEN) == 0) {
				flag_resp += 1;
    			poll_tx_ts[1] = get_tx_timestamp_u64();
    			resp_rx_ts[1] = get_rx_timestamp_u64();
    			dwt_setrxtimeout(2 * ANCHOR_WAIT);
    			dwt_rxenable(DWT_START_RX_IMMEDIATE);
			}
			else if (memcmp(rx_buffer, rx_resp3_msg, ALL_MSG_COMMON_LEN) == 0) {
				flag_resp += 1;
    			poll_tx_ts[2] = get_tx_timestamp_u64();
    			resp_rx_ts[2] = get_rx_timestamp_u64();
    			dwt_setrxtimeout(ANCHOR_WAIT);
    			dwt_rxenable(DWT_START_RX_IMMEDIATE);
			}
			else if (memcmp(rx_buffer, rx_resp4_msg, ALL_MSG_COMMON_LEN) == 0) {
				flag_resp += 1;
    			poll_tx_ts[3] = get_tx_timestamp_u64();
    			resp_rx_ts[3] = get_rx_timestamp_u64();
    			dwt_setrxtimeout(ANCHOR_WAIT);
    			dwt_rxenable(DWT_START_RX_IMMEDIATE);
			}
			// 아무것도 못받은거 -> 없어도 됨
    		else {
    			/* Ranging timing */
    			dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    			dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    			dwt_setpreambledetecttimeout(PRE_TIMEOUT);
    			send_poll(get_random_delay());
    		}
    }
    else if (state == S_FINAL_TX) {

    	frame_len = dwt_getframelength(0);
		if (frame_len <= RX_BUF_LEN)
		{
			dwt_readrxdata(rx_buffer, frame_len, 0);
		}

    /* Check that the frame is the expected response from the companion "DS TWR responder" example.
     * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
		rx_buffer[ALL_MSG_SN_IDX] = 0;
		anchor_id = rx_buffer[5];
		//sprintf(info_str, "anchor ID: %d  ts ok", anchor_id);
		//test_run_info((unsigned char *)info_str);
		if(anchor_id == 1) {
			flag_resp_ts += 1;
			resp_anchor[0] = 1;
			final_tx_ts[0] = get_tx_timestamp_u64();
			final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_rx_ts[0]);
			final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_tx_ts[0]);
			final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_rx_ts[0]);
	        dwt_setrxtimeout(3 * ANCHOR_WAIT);
	        dwt_rxenable(DWT_START_RX_IMMEDIATE);
		}
		else if(anchor_id == 2) {
			flag_resp_ts += 1;
			resp_anchor[1] = 1;
			final_tx_ts[1] = get_tx_timestamp_u64();
			final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_rx_ts[1]);
			final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_tx_ts[1]);
			final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_rx_ts[1]);
	        dwt_setrxtimeout(2 * ANCHOR_WAIT);
	        dwt_rxenable(DWT_START_RX_IMMEDIATE);
		}
		else if(anchor_id == 3) {
			flag_resp_ts += 1;
			resp_anchor[2] = 1;
			final_tx_ts[2] = get_tx_timestamp_u64();
			final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_rx_ts[2]);
			final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_tx_ts[2]);
			final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_rx_ts[2]);
	        dwt_setrxtimeout(ANCHOR_WAIT);
	        dwt_rxenable(DWT_START_RX_IMMEDIATE);
		}
		else if(anchor_id == 4) {
			flag_resp_ts += 1;
			resp_anchor[3] = 1;
			final_tx_ts[3] = get_tx_timestamp_u64();
			final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_rx_ts[3]);
			final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_tx_ts[3]);
			final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_rx_ts[3]);
	        dwt_setrxtimeout(ANCHOR_WAIT);
	        dwt_rxenable(DWT_START_RX_IMMEDIATE);
		}

		else {
			/* Ranging timing */
			dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
			dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
			dwt_setpreambledetecttimeout(PRE_TIMEOUT);
			send_poll(get_random_delay());
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

	//test_run_info((unsigned char *)"flag to");

    if(state == S_POLL_TX) {
    	if(flag_resp < 2) {
    		flag_resp = 0;
            do {
    			dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    			dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    			dwt_setpreambledetecttimeout(PRE_TIMEOUT);
                send_poll(get_random_delay());
            } while (ret != DWT_SUCCESS);
    	}
    	else if(flag_resp >= 2) {
    		flag_resp = 0;
            do {
                send_final();
            } while (ret != DWT_SUCCESS);
    	}
    }

    else if(state == S_FINAL_TX) {
    	if(flag_resp_ts >= 2) {
	        for (int i = 0; i < 4; i++) {
	        	if(resp_anchor[i] == 0)
	        		continue;

	        	Ra[i] = (double)(resp_rx_ts[i] - poll_tx_ts[i]);
	        	Rb[i] = (double)(final_rx_ts[i] - resp_tx_ts[i]);
	        	Da[i] = (double)(final_tx_ts[i] - resp_rx_ts[i]);
	        	Db[i] = (double)(resp_tx_ts[i] - poll_rx_ts[i]);
	        	tof_dtu[i] = (int64_t)((Ra[i] * Rb[i] - Da[i] * Db[i]) / (Ra[i] + Rb[i] + Da[i] + Db[i]));
	        	tof[i] = tof_dtu[i] * DWT_TIME_UNITS;
	        	distances[i] = tof[i] * SPEED_OF_LIGHT;
			    if((distances[i] > 0) && (distances[i] < 1000)){
			    	sprintf(info_str, "distance %d : %3.2f m", i+1, distances[i]);
			        test_run_info((unsigned char *)info_str);
			    }
			    else{
			    	flag_resp_ts = flag_resp_ts - 1;
			    	resp_anchor[i] = 0;
			    }
	        }
    		estimate_tag_position(pos_anchors, distances, num_anchors, flag_resp_ts, resp_anchor);
    	}
    	flag_resp_ts = 0;
    	for(int i = 0; i < num_anchors; i++) {
    		resp_anchor[i] = 0;
    	}
        do {
			dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
			dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
			dwt_setpreambledetecttimeout(PRE_TIMEOUT);
            send_poll(get_random_delay());
        } while (ret != DWT_SUCCESS);
    }

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
    //uint32_t status = dwt_read_reg(0x44UL);
    //sprintf(info_str, "SYS_STATUS: 0x%08X", status);
    // test_run_info((unsigned char *)info_str);
    /* RX 에러(프레임 손상 등): 관련 에러 비트 클리어 */
    dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

   // test_run_info((unsigned char *)"flag err");

    if(state == S_POLL_TX) {
       	if(flag_resp < 2) {
       		flag_resp = 0;
               do {
       			dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
       			dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
       			dwt_setpreambledetecttimeout(PRE_TIMEOUT);
                send_poll(get_random_delay());
               } while (ret != DWT_SUCCESS);
       	}
       	else if(flag_resp >= 2) {
       		flag_resp = 0;
               do {
                   send_final();
               } while (ret != DWT_SUCCESS);
       	}
       }

       else if(state == S_FINAL_TX) {
       	if(flag_resp_ts >= 2) {
   	        for (int i = 0; i < 4; i++) {
   	        	if(resp_anchor[i] == 0)
   	        		continue;

   	        	Ra[i] = (double)(resp_rx_ts[i] - poll_tx_ts[i]);
   	        	Rb[i] = (double)(final_rx_ts[i] - resp_tx_ts[i]);
   	        	Da[i] = (double)(final_tx_ts[i] - resp_rx_ts[i]);
   	        	Db[i] = (double)(resp_tx_ts[i] - poll_rx_ts[i]);
   	        	tof_dtu[i] = (int64_t)((Ra[i] * Rb[i] - Da[i] * Db[i]) / (Ra[i] + Rb[i] + Da[i] + Db[i]));
   	        	tof[i] = tof_dtu[i] * DWT_TIME_UNITS;
   	        	distances[i] = tof[i] * SPEED_OF_LIGHT;
			    if((distances[i] > 0) && (distances[i] < 1000)){
			    	sprintf(info_str, "distance %d : %3.2f m", i+1, distances[i]);
			        //test_run_info((unsigned char *)info_str);
			    }
			    else{
			    	flag_resp_ts = flag_resp_ts - 1;
			    	resp_anchor[i] = 0;
			    }
   	        }
   	        estimate_tag_position(pos_anchors, distances, num_anchors, flag_resp_ts, resp_anchor);
       	}
       	flag_resp_ts = 0;
       	for(int i = 0; i < num_anchors; i++) {
       		resp_anchor[i] = 0;
       	}
           do {
   			dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
   			dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
   			dwt_setpreambledetecttimeout(PRE_TIMEOUT);
            send_poll(get_random_delay());
           } while (ret != DWT_SUCCESS);
       }
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
    (void)cb_data;
    /* This callback has been defined so that a breakpoint can be put here to check it is correctly called but there is actually nothing specific to
     * do on transmission confirmation in this example. Typically, we could activate reception for the response here but this is automatically handled
     * by DW IC using DWT_RESPONSE_EXPECTED parameter when calling dwt_starttx().
     * An actual application that would not need this callback could simply not define it and set the corresponding field to NULL when calling
     * dwt_setcallbacks(). The ISR will not call it which will allow to save some interrupt processing time. */

	// Poll 전송 완료 인터럽트 비트 클리어
	dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

	//test_run_info((unsigned char *)"flag tx_conf");

    if (state == S_POLL_TX) {
    	cb_t3 = dwt_readsystimestamphi32();
    }
	else if (state == S_FINAL_TX) {
		/* Increment frame sequence number after transmission of the final message (modulo 256). */
		frame_seq_nb++;
    }

}

/*---------------------------------------------------
 * Build and send the Poll message
 *---------------------------------------------------*/
static int send_poll(uint32_t poll_delay_uus) {

	start_1 = DWT->CYCCNT;
    // 1) 현재 시스템 타임의 상위 32비트 읽기
    current_time_high32 = dwt_readsystimestamphi32();

    // 2) UWB µs → DWT 내부 단위(≈8 ns)로 환산한 뒤, 상위 32비트값 계산
    delay_high32 = (uint32_t)(((uint64_t)poll_delay_uus * UUS_TO_DWT_TIME) >> 8);

    // 3) 딜레이 예약값(starttime) = “현재 시각(high32) + 딜레이(high32)”
    starttime = current_time_high32 + delay_high32;

    // 4) 예약 송신 시점 설정
    dwt_setdelayedtrxtime(starttime);

    // 5) Poll 메시지 버퍼에 쓰기
    tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
    dwt_writetxfctrl(sizeof(tx_poll_msg) + FCS_LEN, 0, 1);
    // 6) 예약 전송 시작
    ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
    state = S_POLL_TX;
    return ret;

}


/*---------------------------------------------------
 * Build and send the Final message
 *---------------------------------------------------*/
static void send_final(void)
{
    uint32_t final_tx_time;
    latest_rx_time = get_rx_timestamp_u64();
    /* Compute delayed TX for Final */
    final_tx_time = (uint32_t)((latest_rx_time + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8);
    dwt_setdelayedtrxtime(final_tx_time);
	/* Set expected delay and timeout for final message reception. See NOTE 4 and 5 below. */
	dwt_setrxaftertxdelay(FINAL_TX_TO_TS_RX_DLY_UUS);
	dwt_setrxtimeout(TS_RX_TIMEOUT_UUS);

	/* Set preamble timeout for expected frames. See NOTE 6 below. */

	dwt_setpreambledetecttimeout(0);

    tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0);
    dwt_writetxfctrl(sizeof(tx_final_msg) + FCS_LEN, 0, 1);
    /* Send Final */
    ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

    if (ret != DWT_SUCCESS) {
        /* Ranging timing */
        dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
        dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
        dwt_setpreambledetecttimeout(PRE_TIMEOUT);
		do {
			send_poll(get_random_delay());
		} while (ret != DWT_SUCCESS);
    }
    else{
    	state = S_FINAL_TX;
    }

}

static uint32_t get_random_delay(void) {
    return (uint32_t)( (uint32_t)FINAL_TX_TO_POLL_TX_DLY_UUS + rand() % (30000 - (uint32_t)FINAL_TX_TO_POLL_TX_DLY_UUS + 1) );  // FINAL_TX_TO_POLL_TX_DLY_UUS ~ 30000
}

//딜레이 함수
void DWT_Delay_us(volatile uint32_t microseconds) {
    uint32_t clk_cycle_start = DWT->CYCCNT;
    // convert us to clock cycles (SystemCoreClock is in Hz)
    microseconds *= (SystemCoreClock / 1000000);

    while ((DWT->CYCCNT - clk_cycle_start) < microseconds);
}
#endif
