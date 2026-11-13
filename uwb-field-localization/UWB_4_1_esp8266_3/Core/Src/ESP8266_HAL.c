/*
 * ESP8266_DAC_Simplified.c
 * RC카 스타일 제어 수정: FORWARD/BACKWARD 시 1.2V부터 스로틀 점증
 */

#include "UartRingbuffer_multi.h"
#include "ESP8266_HAL.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "ctype.h"
#include "main.h"

extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart2;

extern Point2D pos;
extern double distances[4];
extern Point2D pos_anchors[4];

#define wifi_uart &huart2
#define pc_uart   &huart3

char buffer[20];

void ESP_Init ()
{
    char data[80];

    Ringbuf_init();  // 링버퍼 초기화

    Uart_sendstring("AT+RST\r\n", wifi_uart);
    Uart_sendstring("RESETTING.", pc_uart);
    for (int i = 0; i < 5; i++)
    {
        Uart_sendstring(".", pc_uart);
        HAL_Delay(1000);
    }

    Uart_flush(wifi_uart);
    Uart_sendstring("AT\r\n", wifi_uart);
    while (!(Wait_for("OK\r\n", wifi_uart)));
    Uart_sendstring("AT---->OK\r\n", pc_uart);

    Uart_flush(wifi_uart);
    Uart_sendstring("AT+CWMODE=2\r\n", wifi_uart);
    while (!(Wait_for("OK\r\n", wifi_uart)));
    Uart_sendstring("CW MODE---->2\r\n", pc_uart);

    Uart_flush(wifi_uart);
    Uart_sendstring("AT+CWSAP=\"uwb\",\"12345678\",6,3\r\n", wifi_uart);
    while (!(Wait_for("OK\r\n", wifi_uart)));
    Uart_sendstring("CWSAP---->OK\r\n", pc_uart);

    Uart_flush(wifi_uart);
    Uart_sendstring("AT+CIFSR\r\n", wifi_uart);
    while (!(Wait_for("CIFSR:APIP,\"", wifi_uart)));
    while (!(Copy_upto("\"", buffer, wifi_uart)));
    while (!(Wait_for("OK\r\n", wifi_uart)));
    int len = strlen(buffer);
    buffer[len-1] = '\0';
    sprintf(data, "IP ADDR: %s\r\n", buffer);
    Uart_sendstring(data, pc_uart);

    Uart_flush(wifi_uart);
    Uart_sendstring("AT+CIPMUX=1\r\n", wifi_uart);
    while (!(Wait_for("OK\r\n", wifi_uart)));
    Uart_sendstring("CIPMUX---->OK\r\n", pc_uart);

    Uart_flush(wifi_uart);
    Uart_sendstring("AT+CIPSERVER=1,80\r\n", wifi_uart);
    while (!(Wait_for("OK\r\n", wifi_uart)));
    Uart_sendstring("CIPSERVER---->OK\r\n", pc_uart);

    Uart_sendstring("Now Connect to the IP ADDRESS\r\n", pc_uart);
}

int Server_Send(int Link_ID)
{
	char body[128];
    sprintf(body, "x=%.2f , y=%.2f; dist1: %.2f; dist2: %.2f; dist3: %.2f; dist4: %.2f; %.2f; %.2f ",
    		pos.x, pos.y, distances[0], distances[1], distances[2], distances[3], pos_anchors[3].x, pos_anchors[3].y);
    const char *http_header = "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\n";

    int body_len = strlen(body);
    int header_len = strlen(http_header);
    int total_len = body_len + header_len;

    char command[64];
    sprintf(command, "AT+CIPSEND=%d,%d\r\n", Link_ID, total_len);
    wifi_send(command);
    //Uart_sendstring(command, wifi_uart);
    if (!Wait_for(">", wifi_uart)) {
    	test_run_info("Failed: Wait for >\r\n");
    	//Uart_sendstring("Failed: Wait for >\r\n", pc_uart);
        return 0;
    }
    wifi_send(http_header);
    wifi_send(body);
    //Uart_sendstring(http_header, wifi_uart);
    //Uart_sendstring(body, wifi_uart);
    if (!Wait_for("SEND OK", wifi_uart)) {
    	test_run_info("Failed: Wait for SEND OK\r\n");
    	//Uart_sendstring("Failed: Wait for SEND OK\r\n", pc_uart);
        return 0;
    }
    sprintf(command, "AT+CIPCLOSE=%d\r\n", Link_ID);
    wifi_send(command);
    //Uart_sendstring(command, wifi_uart);
    if (!Wait_for("OK\r\n", wifi_uart)) {
    	test_run_info("Failed: Wait for CIPCLOSE OK");
    	//Uart_sendstring("Failed: Wait for CIPCLOSE OK\r\n", pc_uart);
    }
    return 1;
}

void Server_Start(void)
{
    static char buftocopyinto[64] = {0};
    static char Link_ID = -1;
    static int isProcessing = 0; // 요청 처리 중 여부 플래그
//    static uint32_t last_request_time = 0; // 마지막 요청 시간

    // 이미 처리 중이거나 너무 짧은 간격이면 무시
    if (isProcessing) return;
    // 요청이 있는지 확인 (비블록킹)
    if (IsDataAvailable(wifi_uart) && Get_after("+IPD,", 1, &Link_ID, wifi_uart))
    {
        isProcessing = 1; // 처리 시작
        Link_ID -= '0';

        memset(buftocopyinto, 0, sizeof(buftocopyinto));
        if (Copy_upto(" HTTP/1.1", buftocopyinto, wifi_uart))
        {
            char debug_msg[128];
            sprintf(debug_msg, "Received Request: %s\r\n", buftocopyinto);
            test_run_info(debug_msg);
            Server_Send(Link_ID);
            //Uart_sendstring(debug_msg, pc_uart);
        }
        isProcessing = 0; // 처리 완료
        Uart_flush(wifi_uart); // 버퍼 비우기
    }
}
