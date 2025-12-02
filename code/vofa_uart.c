#include "vofa_uart.h"
#include "vofa_function.h"
#include "zf_device_wireless_uart.h"
#include "zf_driver_uart.h"


uint8_t rx_data = 0;
uint8_t            vofaRxBufferIndex = 0;
extern vofaCommand cmd;

void uartSendByte(const uint8_t c)
{
	wireless_uart_send_byte(c); //淇敼涓轰覆鍙ｅ彂閫佹帴鍙�
	uart_write_byte(UART_0, c);
}

void uartSendData(uint8_t* Array, uint8_t SIZE)
{
	while (SIZE)
	{
		uartSendByte(*Array);
		Array++;
		SIZE--;
	}
}
