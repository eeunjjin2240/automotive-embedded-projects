/*
 * UartRingbuffer.c
 *
 *  Created on: 10-Jul-2019
 *      Author: Controllerstech
 */

#include "UartRingbuffer_multi.h"
#include <string.h>


/*  Define the device uart and pc uart below according to your setup  */

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

#define device_uart &huart2
#define pc_uart &huart3

/* put the following in the ISR

extern void Uart_isr (UART_HandleTypeDef *huart);

*/

/**************** =====================================>>>>>>>>>>>> NO chnages after this **********************/


ring_buffer rx_buffer1 = { { 0 }, 0, 0};
ring_buffer tx_buffer1 = { { 0 }, 0, 0};
ring_buffer rx_buffer2 = { { 0 }, 0, 0};
ring_buffer tx_buffer2 = { { 0 }, 0, 0};

ring_buffer *_rx_buffer1;
ring_buffer *_tx_buffer1;
ring_buffer *_rx_buffer2;
ring_buffer *_tx_buffer2;

void store_char (unsigned char c, ring_buffer *buffer);

// 내부 링버퍼 구조체를 초기화하고 uart 인터럽트를 활성화
void Ringbuf_init(void)
{
  _rx_buffer1 = &rx_buffer1;
  _tx_buffer1 = &tx_buffer1;
  _rx_buffer2 = &rx_buffer2;
  _tx_buffer2 = &tx_buffer2;

  /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
  __HAL_UART_ENABLE_IT(device_uart, UART_IT_ERR);
  __HAL_UART_ENABLE_IT(pc_uart, UART_IT_ERR);

  /* Enable the UART Data Register not empty Interrupt */
  __HAL_UART_ENABLE_IT(device_uart, UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(pc_uart, UART_IT_RXNE);
}

// 받은 문자를 링버퍼에 저장
void store_char(unsigned char c, ring_buffer *buffer)
{
  int i = (unsigned int)(buffer->head + 1) % UART_BUFFER_SIZE;

  // if we should be storing the received character into the location
  // just before the tail (meaning that the head would advance to the
  // current location of the tail), we're about to overflow the buffer
  // and so we don't write the character or advance the head.
  if(i != buffer->tail) {
    buffer->buffer[buffer->head] = c;
    buffer->head = i;
  }
}

// 특정 문자열이 버퍼 내에 존재하는지 검사
int Look_for (char *str, char *buffertolookinto)
{
	int stringlength = strlen (str);
	int bufferlength = strlen (buffertolookinto);
	int so_far = 0;
	int indx = 0;
repeat:
	while (str[so_far] != buffertolookinto[indx]) indx++;
	if (str[so_far] == buffertolookinto[indx]){
	while (str[so_far] == buffertolookinto[indx])
	{
		so_far++;
		indx++;
	}
	}

	else
		{
			so_far =0;
			if (indx >= bufferlength) return -1;
			goto repeat;
		}

	if (so_far == stringlength) return 1;
	else return -1;
}

// startStirng과 endString 사이의 데이터를 추출
void GetDataFromBuffer (char *startString, char *endString, char *buffertocopyfrom, char *buffertocopyinto)
{
	int startStringLength = strlen (startString);
	int endStringLength   = strlen (endString);
	int so_far = 0;
	int indx = 0;
	int startposition = 0;
	int endposition = 0;

repeat1:
	while (startString[so_far] != buffertocopyfrom[indx]) indx++;
	if (startString[so_far] == buffertocopyfrom[indx])
	{
		while (startString[so_far] == buffertocopyfrom[indx])
		{
			so_far++;
			indx++;
		}
	}

	if (so_far == startStringLength) startposition = indx;
	else
	{
		so_far =0;
		goto repeat1;
	}

	so_far = 0;

repeat2:
	while (endString[so_far] != buffertocopyfrom[indx]) indx++;
	if (endString[so_far] == buffertocopyfrom[indx])
	{
		while (endString[so_far] == buffertocopyfrom[indx])
		{
			so_far++;
			indx++;
		}
	}

	if (so_far == endStringLength) endposition = indx-endStringLength;
	else
	{
		so_far =0;
		goto repeat2;
	}

	so_far = 0;
	indx=0;

	for (int i=startposition; i<endposition; i++)
	{
		buffertocopyinto[indx] = buffertocopyfrom[i];
		indx++;
	}
}

// 지정한 uart의 수신 버퍼 초기화
void Uart_flush (UART_HandleTypeDef *uart)
{
	if (uart == device_uart)
	{
		memset(_rx_buffer1->buffer,'\0', UART_BUFFER_SIZE);
		_rx_buffer1->head = 0;
	}
	if (uart == pc_uart)
	{
		memset(_rx_buffer2->buffer,'\0', UART_BUFFER_SIZE);
		_rx_buffer2->head = 0;
	}
}

// 수신 버퍼의 현재 tail 위치의 문자 반환 (읽지 않고 미리 보기)
int Uart_peek(UART_HandleTypeDef *uart)
{
	if (uart == device_uart)
	{
		  if(_rx_buffer1->head == _rx_buffer1->tail)
		  {
		    return -1;
		  }
		  else
		  {
		    return _rx_buffer1->buffer[_rx_buffer1->tail];
		  }
	}

	else if (uart == pc_uart)
	{
		  if(_rx_buffer2->head == _rx_buffer2->tail)
		  {
		    return -1;
		  }
		  else
		  {
		    return _rx_buffer2->buffer[_rx_buffer2->tail];
		  }
	}

	return -1;
}

// 버퍼에서 하나의 문자 읽고 다음으로 tail 이동
int Uart_read(UART_HandleTypeDef *uart)
{
	if (uart == device_uart)
	{
		  // if the head isn't ahead of the tail, we don't have any characters
		  if(_rx_buffer1->head == _rx_buffer1->tail)
		  {
		    return -1;
		  }
		  else
		  {
		    unsigned char c = _rx_buffer1->buffer[_rx_buffer1->tail];
		    _rx_buffer1->tail = (unsigned int)(_rx_buffer1->tail + 1) % UART_BUFFER_SIZE;
		    return c;
		  }
	}

	else if (uart == pc_uart)
	{
		  // if the head isn't ahead of the tail, we don't have any characters
		  if(_rx_buffer2->head == _rx_buffer2->tail)
		  {
		    return -1;
		  }
		  else
		  {
		    unsigned char c = _rx_buffer2->buffer[_rx_buffer2->tail];
		    _rx_buffer2->tail = (unsigned int)(_rx_buffer2->tail + 1) % UART_BUFFER_SIZE;
		    return c;
		  }
	}

	else return -1;
}

// uart 송신 버퍼에 공간이 생길 때까지 대기, 문자를 버퍼에 넣고 uart tx 인터럽트 활성화
void Uart_write(int c, UART_HandleTypeDef *uart)
{
	if (c>=0)
	{
		if (uart == device_uart){
		int i = (_tx_buffer1->head + 1) % UART_BUFFER_SIZE;

		// If the output buffer is full, there's nothing for it other than to
		// wait for the interrupt handler to empty it a bit
		// ???: return 0 here instead?
		while (i == _tx_buffer1->tail);

		_tx_buffer1->buffer[_tx_buffer1->head] = (uint8_t)c;
		_tx_buffer1->head = i;
		__HAL_UART_ENABLE_IT(device_uart, UART_IT_TXE); // Enable UART transmission interrupt
		}

		else if (uart == pc_uart){
			int i = (_tx_buffer2->head + 1) % UART_BUFFER_SIZE;

			// If the output buffer is full, there's nothing for it other than to
			// wait for the interrupt handler to empty it a bit
			// ???: return 0 here instead?
			while (i == _tx_buffer2->tail);
			_tx_buffer2->buffer[_tx_buffer2->head] = (uint8_t)c;
			_tx_buffer2->head = i;
			__HAL_UART_ENABLE_IT(pc_uart, UART_IT_TXE); // Enable UART transmission interrupt

		}
	}
}

// 지정한 uart의 수신 버퍼에 데이터가 몇 개 들어있는지 반환(링버퍼 내에 데이터 개수 반환)
int IsDataAvailable(UART_HandleTypeDef *uart)
{
	if (uart == device_uart) return (uint16_t)(UART_BUFFER_SIZE + _rx_buffer1->head - _rx_buffer1->tail) % UART_BUFFER_SIZE;
	else if (uart == pc_uart) return (uint16_t)(UART_BUFFER_SIZE + _rx_buffer2->head - _rx_buffer2->tail) % UART_BUFFER_SIZE;
	return -1;
}

// 특정 문자열 이후 지정된 개수만큼 문자 읽어오기
int Get_after (char *string, uint8_t numberofchars, char *buffertosave, UART_HandleTypeDef *uart)
{

	while (Wait_for(string, uart) != 1);
	for (int indx=0; indx<numberofchars; indx++)
	{
		while (!(IsDataAvailable(uart)));
		buffertosave[indx] = Uart_read(uart);
	}
	return 1;
}

// 문자열을 uart 송신 버퍼로 전송
void Uart_sendstring (const char *s, UART_HandleTypeDef *uart)
{
	while(*s!='\0') Uart_write(*s++, uart);
}

// 숫자를 지정된 진수 문자열로 변환해 uart로 출력
void Uart_printbase (long n, uint8_t base, UART_HandleTypeDef *uart)
{
  char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
  char *s = &buf[sizeof(buf) - 1];

  *s = '\0';

  // prevent crash if called with base == 1
  if (base < 2) base = 10;

  do {
    unsigned long m = n;
    n /= base;
    char c = m - base * n;
    *--s = c < 10 ? c + '0' : c + 'A' - 10;
  } while(n);

  while(*s) Uart_write(*s++, uart);
}

// 특정 문자열이 나올때까지 버퍼에서 읽어서 buffertocopyinto에 복사
int Copy_upto (char *string, char *buffertocopyinto, UART_HandleTypeDef *uart)
{
    int so_far = 0;
    int len = strlen(string);
    int indx = 0;

    ring_buffer *rx_buffer = (uart == &huart2) ? _rx_buffer1 :
                              (uart == &huart3) ? _rx_buffer2 : NULL;
    if (!rx_buffer) return -1;

again:
    while (!IsDataAvailable(uart));
    while (Uart_peek(uart) != string[so_far]) {
        buffertocopyinto[indx] = rx_buffer->buffer[rx_buffer->tail];
        rx_buffer->tail = (rx_buffer->tail + 1) % UART_BUFFER_SIZE;
        indx++;
        while (!IsDataAvailable(uart));
    }

    while (Uart_peek(uart) == string[so_far]) {
        so_far++;
        buffertocopyinto[indx++] = Uart_read(uart);
        if (so_far == len) return 1;
        while (!IsDataAvailable(uart));
    }

    if (so_far != len) {
        so_far = 0;
        goto again;
    }
    return (so_far == len) ? 1 : -1;
}

// 지정된 문자열이 수신될 때까지 대기
int Wait_for (char *string, UART_HandleTypeDef *uart)
{
    int so_far = 0;
    int len = strlen(string);

    ring_buffer *rx_buffer = (uart == &huart2) ? _rx_buffer1 :
                              (uart == &huart3) ? _rx_buffer2 : NULL;
    if (!rx_buffer) return -1;

again:
    while (!IsDataAvailable(uart));
    if (Uart_peek(uart) != string[so_far]) {
        rx_buffer->tail = (rx_buffer->tail + 1) % UART_BUFFER_SIZE;
        goto again;
    }
    while (Uart_peek(uart) == string[so_far]) {
        so_far++;
        Uart_read(uart);
        if (so_far == len) return 1;
        while (!IsDataAvailable(uart));
    }

    if (so_far != len) {
        so_far = 0;
        goto again;
    }
    return (so_far == len) ? 1 : -1;
}

// uart 인터럽트 서비스 루틴 -> rx시, store 호출로 링버퍼에 저장 / tx시, 송신 버퍼에 데이터 있으면 다음 문자 송신 없으면 인터럽트 비활성화
void Uart_isr (UART_HandleTypeDef *huart)
{
	  uint32_t isrflags   = READ_REG(huart->Instance->SR);
	  uint32_t cr1its     = READ_REG(huart->Instance->CR1);

    /* if DR is not empty and the Rx Int is enabled */
    if (((isrflags & USART_SR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
    {
    	 /******************
    	    	      *  @note   PE (Parity error), FE (Framing error), NE (Noise error), ORE (Overrun
    	    	      *          error) and IDLE (Idle line detected) flags are cleared by software
    	    	      *          sequence: a read operation to USART_SR register followed by a read
    	    	      *          operation to USART_DR register.
    	    	      * @note   RXNE flag can be also cleared by a read to the USART_DR register.
    	    	      * @note   TC flag can be also cleared by software sequence: a read operation to
    	    	      *          USART_SR register followed by a write operation to USART_DR register.
    	    	      * @note   TXE flag is cleared only by a write to the USART_DR register.

    	 *********************/
		huart->Instance->SR;                       /* Read status register */
        unsigned char c = huart->Instance->DR;     /* Read data register */
        if (huart == device_uart)
        {
        	store_char (c, _rx_buffer1);  // store data in buffer
        }

        else if (huart == pc_uart)
        {
           	store_char (c, _rx_buffer2);  // store data in buffer
        }

        return;
    }

    /*If interrupt is caused due to Transmit Data Register Empty */
    if (((isrflags & USART_SR_TXE) != RESET) && ((cr1its & USART_CR1_TXEIE) != RESET))
    {
    	if (huart == device_uart){
    	if(tx_buffer1.head == tx_buffer1.tail)
    	    {
    	      // Buffer empty, so disable interrupts
    	      __HAL_UART_DISABLE_IT(huart, UART_IT_TXE);

    	    }

    	 else
    	    {
    	      // There is more data in the output buffer. Send the next byte
    	      unsigned char c = tx_buffer1.buffer[tx_buffer1.tail];
    	      tx_buffer1.tail = (tx_buffer1.tail + 1) % UART_BUFFER_SIZE;

    	      /******************
    	      *  @note   PE (Parity error), FE (Framing error), NE (Noise error), ORE (Overrun
    	      *          error) and IDLE (Idle line detected) flags are cleared by software
    	      *          sequence: a read operation to USART_SR register followed by a read
    	      *          operation to USART_DR register.
    	      * @note   RXNE flag can be also cleared by a read to the USART_DR register.
    	      * @note   TC flag can be also cleared by software sequence: a read operation to
    	      *          USART_SR register followed by a write operation to USART_DR register.
    	      * @note   TXE flag is cleared only by a write to the USART_DR register.

    	      *********************/

    	      huart->Instance->SR;
    	      huart->Instance->DR = c;

    	    }
    	}

    	else if (huart == pc_uart){
        	if(tx_buffer2.head == tx_buffer2.tail)
        	    {
        	      // Buffer empty, so disable interrupts
        	      __HAL_UART_DISABLE_IT(huart, UART_IT_TXE);

        	    }

        	 else
        	    {
        	      // There is more data in the output buffer. Send the next byte
        	      unsigned char c = tx_buffer2.buffer[tx_buffer2.tail];
        	      tx_buffer2.tail = (tx_buffer2.tail + 1) % UART_BUFFER_SIZE;

        	      /******************
        	      *  @note   PE (Parity error), FE (Framing error), NE (Noise error), ORE (Overrun
        	      *          error) and IDLE (Idle line detected) flags are cleared by software
        	      *          sequence: a read operation to USART_SR register followed by a read
        	      *          operation to USART_DR register.
        	      * @note   RXNE flag can be also cleared by a read to the USART_DR register.
        	      * @note   TC flag can be also cleared by software sequence: a read operation to
        	      *          USART_SR register followed by a write operation to USART_DR register.
        	      * @note   TXE flag is cleared only by a write to the USART_DR register.

        	      *********************/

        	      huart->Instance->SR;
        	      huart->Instance->DR = c;

        	    }
        	}
    	return;
    }
}

/*** Depreciated For now. This is not needed, try using other functions to meet the requirement ***/
/* get the position of the given string within the incoming data.
 * It returns the position, where the string ends
 */
/*
int16_t Get_position (char *string, UART_HandleTypeDef *uart)
{
  static uint8_t so_far;
  uint16_t counter;
  int len = strlen (string);
  if (uart == device_uart)
  {
	 if (_rx_buffer1->tail>_rx_buffer1->head)
	 {
	  if (Uart_read(device_uart) == string[so_far])
	  		{
	  		  counter=UART_BUFFER_SIZE-1;
	  		  so_far++;
	  		}
	  else so_far=0;
	 }
	 unsigned int start = _rx_buffer1->tail;
	 unsigned int end = _rx_buffer1->head;
	 for (unsigned int i=start; i<end; i++)
	 {
	  if (Uart_read(device_uart) == string[so_far])
		{
		  counter=i;
		  so_far++;
		}
	  else so_far =0;
	 }
  }
  else if (uart == pc_uart)
  {
	 if (_rx_buffer2->tail>_rx_buffer2->head)
	 {
	  if (Uart_read(pc_uart) == string[so_far])
	  		{
	  		  counter=UART_BUFFER_SIZE-1;
	  		  so_far++;
	  		}
	  else so_far=0;
	 }
	 unsigned int start = _rx_buffer2->tail;
	 unsigned int end = _rx_buffer2->head;
	 for (unsigned int i=start; i<end; i++)
	 {
	  if (Uart_read(pc_uart) == string[so_far])
		{
		  counter=i;
		  so_far++;
		}
	  else so_far =0;
	 }
  }

	 if (so_far == len)
	 {
	  so_far =0;
	  return counter;
	 }
	 else return -1;
}
*/
