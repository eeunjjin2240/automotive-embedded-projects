/*
 * keypad.c
 *
 *  Created on: Oct 6, 2025
 *      Author: eunji
 */


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stm32f4xx_hal.h>
#include <main.h>
#include "keypad.h"

extern void test_run_info(unsigned char *data);

#define APP_NAME            "KEYPAD"

// 행,열 핀과 포트
const uint16_t ROW_PINS[4] = {ROW1_Pin, ROW2_Pin, ROW3_Pin, ROW4_Pin};
GPIO_TypeDef* ROW_PORTS[4] = {ROW1_GPIO_Port, ROW2_GPIO_Port, ROW3_GPIO_Port, ROW4_GPIO_Port};

const uint16_t COL_PINS[4] = {COL1_Pin, COL2_Pin, COL3_Pin, COL4_Pin};
GPIO_TypeDef* COL_PORTS[4] = {COL1_GPIO_Port, COL2_GPIO_Port, COL3_GPIO_Port, COL4_GPIO_Port};

// ===== 7-세그 핀 매핑 =====
// 세그먼트 순서: a b c d e f g dp
const uint16_t SEG_PINS[8] = {
		SEGA_Pin, SEGB_Pin, SEGC_Pin, SEGD_Pin, SEGE_Pin, SEGF_Pin, SEGG_Pin, SEGDP_Pin
};

GPIO_TypeDef* SEG_PORTS[8] = {
		SEGA_GPIO_Port, SEGB_GPIO_Port, SEGC_GPIO_Port, SEGD_GPIO_Port, SEGE_GPIO_Port, SEGF_GPIO_Port, SEGG_GPIO_Port, SEGDP_GPIO_Port
};

// 자릿수 순서: DIG0(천) DIG1(백) DIG2(십) DIG3(일)
const uint16_t DIG_PINS [4] = { DIGIT1_Pin, DIGIT2_Pin, DIGIT3_Pin, DIGIT4_Pin };
GPIO_TypeDef* DIG_PORTS[4] = { DIGIT1_GPIO_Port, DIGIT2_GPIO_Port, DIGIT3_GPIO_Port, DIGIT4_GPIO_Port };

// 키맵
char KEYMAP[4][4] = {
	{'1','2','3','A'},
	{'4','5','6','B'},
	{'7','8','9','C'},
	{'*','0','#','D'}
};

const uint8_t SEG_MAP[16] = {
		// 0~9, A,b,C,d,*(-),#(-)
		0b00111111, 0b00000110, 0b01011011, 0b01001111, 0b01100110, 0b01101101, 0b01111101, 0b00000111,
		0b01111111, 0b01101111, 0b01110111, 0b01111100, 0b00111001, 0b01011110, 0b01000000, 0b01000000
};

char key;
static volatile uint8_t display_enabled = 1;  // 깜빡임 OFF 때 0

void buzzer_on (void){ HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET); }
void buzzer_off(void){ HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET); }

// === EXTI→메인 전달 플래그 ===
static volatile char pending_key = 0;
static volatile uint8_t key_ready = 0;

// === 입력 카운트/패스코드 ===
static uint8_t push_cnt = 0;
static const uint8_t PASS[4] = {2,5,8,0};

// 표시 버퍼(0~15 값 저장: 숫자/헥사/문자 일부)
volatile uint8_t disp[4] = {0,0,0,0};
volatile uint8_t cur_digit = 0;

typedef struct {
  const uint16_t* steps_ms;  // 각 구간 길이(ms)
  const uint8_t*  steps_on;  // ON(1)/OFF(0)
  uint8_t len, idx, active;
  uint32_t next_ms;
} buzzer_seq_t;

static buzzer_seq_t bz = {0};

typedef struct {
  uint8_t repeats; uint8_t state_on; uint8_t active;
  uint16_t period_ms; uint32_t next_ms;
} blink_seq_t;

static blink_seq_t bl = {0};

// 패턴
static const uint16_t P_OK_T[2]  = { 600, 40 };                 // 성공: 삐— 600ms
static const uint8_t  P_OK_ON[2] = { 1, 1 };

static const uint16_t P_NG_T[]  = {100,80,100,80,100};     // 실패: 삐 100×3
static const uint8_t  P_NG_ON[] = {  1, 0,  1, 0,  1};


static int pass_matches(void){
  return (disp[0]==PASS[0] && disp[1]==PASS[1] && disp[2]==PASS[2] && disp[3]==PASS[3]);
}

void SEG_CLEAR(void){
	for (int i=0;i<8;i++)
		HAL_GPIO_WritePin(SEG_PORTS[i], SEG_PINS[i], GPIO_PIN_RESET); // CC: LOW=OFF
}
void DIG_OFF_ALL(void){
	for (int i=0;i<4;i++)
		HAL_GPIO_WritePin(DIG_PORTS[i], DIG_PINS[i], GPIO_PIN_SET);   // CC: HIGH=OFF
}
static inline void DIG_ON(int d){
	HAL_GPIO_WritePin(DIG_PORTS[d], DIG_PINS[d], GPIO_PIN_RESET); // CC: LOW=ON
}
static inline void SEG_WRITE(uint8_t code){
	for (int i=0;i<8;i++){
		GPIO_PinState s = (code & (1<<i)) ? GPIO_PIN_SET : GPIO_PIN_RESET; // CC: 1=ON → HIGH
		HAL_GPIO_WritePin(SEG_PORTS[i], SEG_PINS[i], s);
	}
}

// 버퍼에 오른쪽 끝 digit를 푸시 (스크롤)
static inline void disp_push(uint8_t v){
	disp[0]=disp[1]; disp[1]=disp[2]; disp[2]=disp[3]; disp[3]=v & 0x0F;
	push_cnt++;
	if(push_cnt == 4)
		push_cnt = 0;
}

static void rows_all_high(void) {
	for (int i=0; i<4; i++)
		HAL_GPIO_WritePin(ROW_PORTS[i], ROW_PINS[i], GPIO_PIN_SET);
	//test_run_info("ROW ALL HIGH");
}

static void rows_all_low(void) {
	for (int i=0; i<4; i++)
		HAL_GPIO_WritePin(ROW_PORTS[i], ROW_PINS[i], GPIO_PIN_RESET);
	//test_run_info("ROW ALL RESET");
}

static void set_row_low(int r) {
	HAL_GPIO_WritePin(ROW_PORTS[r], ROW_PINS[r], GPIO_PIN_RESET);
}

// 인터럽트에서 호출: (row, col) 찾아 문자 리턴. 못 찾으면 0
char scan_once_get_key(void){
	// 행 모두 HIGH로 만들고, 한 행씩 LOW로 내려보며 열을 읽음
	for (int r=0; r<4; r++){
		rows_all_high();
		set_row_low(r);

		for (int c=0; c<4; c++){
			if (HAL_GPIO_ReadPin(COL_PORTS[c], COL_PINS[c]) == GPIO_PIN_RESET){
				key = KEYMAP[r][c];
                char msg[2] = { key, '\0' };
                test_run_info((unsigned char*)msg);
				rows_all_low();  // 유휴 복귀
				return key;
			}
	    }
	}
	rows_all_low();
	return 0;
}

static uint8_t keychar_to_segval(char k){
	// 0~9
	if (k>='0' && k<='9')
		return (uint8_t)(k - '0');
	// A,B,C,D,* ,# → 세그 값에 맞게 매핑 (원하면 다른 동작)
	switch(k){
        case 'A': return 0x0A;
        case 'B': return 0x0B;
        case 'C': return 0x0C;
        case 'D': return 0x0D;
        case '*': return 0x0E;
        case '#': return 0x0F;
        default: return 0;
	}
}

int keypad(void) {
	test_run_info((unsigned char *)APP_NAME);
	rows_all_low(); // 초기 전원 인가 후 안전 상태
}

void buzzer_start(const uint16_t* t_ms, const uint8_t* on, uint8_t n){
	bz.steps_ms = t_ms;
	bz.steps_on = on;
	bz.len = n;
	bz.idx = 0;
	bz.active = 1;
	bz.next_ms = HAL_GetTick();
}

void buzzer_task(void){
	if (!bz.active)
		return;
	uint32_t now = HAL_GetTick();
	if ((int32_t)(now - bz.next_ms) < 0)
		return;

	if (bz.idx < bz.len) {
		if (bz.steps_on[bz.idx])
			buzzer_on();
		else
			buzzer_off();
		bz.next_ms = now + bz.steps_ms[bz.idx];
		bz.idx++;
	}
	else {
		buzzer_off();
		bz.active = 0;
	}
}

static void blink_start(uint8_t times, uint16_t period_ms){
	bl.repeats = times * 2;  // ON/OFF 쌍 수
	bl.state_on = 0;         // OFF부터 시작
	bl.period_ms = period_ms;
	bl.next_ms = HAL_GetTick();
	bl.active = 1;
}

void blink_task(void){
	if (!bl.active)
		return;
	uint32_t now = HAL_GetTick();
	if ((int32_t)(now - bl.next_ms) < 0)
		return;

	bl.state_on = !bl.state_on;
	display_enabled = bl.state_on ? 1 : 0;
	bl.next_ms = now + bl.period_ms;

	if (bl.repeats)
		bl.repeats--;
	if (bl.repeats == 0) {
		bl.active = 0;
		display_enabled = 1;
	}
}

void keypad_task(void){
	if (!key_ready) return;

	char k = pending_key;
	key_ready = 0;
	pending_key = 0;

	if (k != '*') {
		uint8_t v = keychar_to_segval(k);
		disp_push(v);
		buzzer_start((P_OK_T+1), (P_OK_ON+1), 1);
		push_cnt = (push_cnt + 1) % 4;
		// (원하면 짧은 클릭음 패턴 따로 만들어 buzzer_start 호출)
	}
	else {
		// '*' 입력 시 판정 로직
		if ((push_cnt % 4) == 0) {
			if (pass_matches()) {
				// 성공: 긴 삐— + 화면 잠깐 끄고 리셋
				buzzer_start(P_OK_T, P_OK_ON, 1);
				blink_start(1, 600);          // 1회 깜빡 (원하면)
			}
			else {
				// 실패: 삐삐삐 + 3회 깜빡
				buzzer_start(P_NG_T, P_NG_ON, (uint8_t)(sizeof(P_NG_T)/sizeof(P_NG_T[0])));
				blink_start(3, 120);
			}
		}
		else {
			// 4자리 안 찼는데 '*' → 실패 패턴
			buzzer_start(P_NG_T, P_NG_ON, (uint8_t)(sizeof(P_NG_T)/sizeof(P_NG_T[0])));
			blink_start(3, 120);
		}
		DIG_OFF_ALL();
		disp[0]=disp[1]=disp[2]=disp[3]=0;
	}
}

// 키패드 인터럽트 콜백 함수
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

    switch (GPIO_Pin)
    {
        /*case DW_RESET_Pin:
            signalResetDone = 1;
            break;

        case DW_IRQn_Pin :
        case DW_IRQ2_Pin :
        {
            //while (HAL_GPIO_ReadPin(DECAIRQ_GPIO, DW_IRQn_Pin) == GPIO_PIN_SET)
            {
                process_deca_irq();
                //dwt_isr();
            }

            break;
        }*/
        case COL1_Pin:
        case COL2_Pin:
        case COL3_Pin:
        case COL4_Pin:
        {
            // 1) 스캔 동안 그룹 마스크
            HAL_NVIC_DisableIRQ(EXTI4_IRQn);
            HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
            HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);

            char k = scan_once_get_key();          // 딜레이 없는 버전
            if (!key_ready && k){
                pending_key = k;
                key_ready = 1;
            }

            __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
            // 4) 재허용
            HAL_NVIC_EnableIRQ(EXTI4_IRQn);
            HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
            HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
            break;
        }

        default:
            break;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance == TIM3){
		static uint8_t cur_digit = 0;

		// 깜빡임 OFF 상태면 모두 끄고 리턴
		if (!display_enabled){
			DIG_OFF_ALL();                         // CC: HIGH=OFF
			return;
		}
		DIG_OFF_ALL();                           // 이전 자리 OFF
		uint8_t code = SEG_MAP[ disp[cur_digit] & 0x0F ];
		SEG_WRITE(code);                         // 세그먼트 데이터 출력
		DIG_ON(cur_digit);                       // 현재 자리 ON (CC: LOW=ON)
		cur_digit = (cur_digit + 1) & 0x03;      // 다음 자리
	}
}
