/**
 * \file
 *
 * \brief SD/MMC card example with FatFs
 *
 * Copyright (c) 2012-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

/**
 * \mainpage SD/MMC/SDIO Card with FatFs Example
 *
 * \section Purpose
 *
 * This example shows how to implement the SD/MMC stack with the FatFS.
 * It will mount the file system and write a file in the card.
 *
 * The example outputs the information through the standard output (stdio).
 * To know the output used on the board, look in the conf_example.h file
 * and connect a terminal to the correct port.
 *
 * While using Xplained Pro evaluation kits, please attach I/O1 Xplained Pro
 * extension board to EXT1.
 *
 * \section Usage
 *
 * -# Build the program and download it into the board.
 * -# On the computer, open and configure a terminal application.
 * Refert to conf_example.h file.
 * -# Start the application.
 * -# In the terminal window, the following text should appear:
 *    \code
	-- SD/MMC/SDIO Card Example on FatFs --
	-- Compiled: xxx xx xxxx xx:xx:xx --
	Please plug an SD, MMC or SDIO card in slot.
\endcode
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
/*
#include <asf.h>
#include "conf_example.h"
#include <string.h>

/**
 * \brief Application entry point.
 *
 * \return Unused (ANSI-C compatibility).
 *//*
int main(void)
{
	char test_file_name[] = "my_second.txt", buffer[10];
	Ctrl_status status;
	FRESULT res;
	FATFS fs;
	FIL file_object;
	UINT* count = 0;
	const usart_serial_options_t usart_serial_options = {
		.baudrate   = CONF_TEST_BAUDRATE,
		.charlength = CONF_TEST_CHARLENGTH,
		.paritytype = CONF_TEST_PARITY,
		.stopbits   = CONF_TEST_STOPBITS,
	};

	irq_initialize_vectors();
	cpu_irq_enable();

	sysclk_init();
	board_init();
	stdio_serial_init(CONF_TEST_USART, &usart_serial_options);

	/* Initialize SD MMC stack 
	sd_mmc_init();

	while (1) {
		/* Wait card present and ready 
		do {
			status = sd_mmc_test_unit_ready(0);
			if (CTRL_FAIL == status) {
				printf("Card install FAIL\n\r");
				printf("Please unplug and re-plug the card.\n\r");
				while (CTRL_NO_PRESENT != sd_mmc_check(0)) {
				}
			}
		} while (CTRL_GOOD != status);

		memset(&fs, 0, sizeof(FATFS));
		res = f_mount(LUN_ID_SD_MMC_0_MEM, &fs);
		if (FR_INVALID_DRIVE == res) {
			printf("[FAIL] res %d\r\n", res);
			goto main_end_of_test;
		}
		printf("[OK]\r\n");

		printf("Create a file (f_open)...\r\n");
//		test_file_name[0] = LUN_ID_SD_MMC_0_MEM + '0';
		res = f_open(&file_object,
				(char const *)test_file_name,
				FA_CREATE_ALWAYS | FA_WRITE);
		if (res != FR_OK) {
			printf("[FAIL] res %d\r\n", res);
			goto main_end_of_test;
		}
		printf("[OK]\r\n");

		printf("Write to test file (f_puts)...\r\n");
		if (0 == f_puts("Кракозябры\n", &file_object)) {
			f_close(&file_object);
			printf("[FAIL]\r\n");
			goto main_end_of_test;
		}
		f_close(&file_object);
		
		res = f_open(&file_object,
		(char const *)test_file_name, FA_READ);
		if (res != FR_OK) {
			printf("[FAIL] res %d\r\n", res);
			goto main_end_of_test;
		}
		printf("[OK]\r\n");
		f_gets (buffer, 10, &file_object);
		f_read (&file_object, buffer, 10, count);
		printf("[OK]\r\n");
		f_close(&file_object);
		printf("Test is successful.\n\r");	
		
main_end_of_test:
		printf("Please unplug the card.\n\r");
		while (CTRL_NO_PRESENT != sd_mmc_check(0)) {
		}
	}
}
*/
		
#include <asf.h>
#include <string.h>
#include "stdio_serial.h"
#include "conf_board.h"
#include "led.h"
#include "BNO055.h"


/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond

/** Wait Time */
#define WAIT_TIME   10
/** TWI Bus Clock 400kHz */
#define TWIHS_CLK     400000

enum setup_bno055
{
	ndof,
	amg
	}register_value;

enum registr_write{
	opr
	}registr_w;

enum registr_read{
	temp,
	pitch_m,
	pitch_l,
	roll_m,
	roll_l,
	heading_m,
	heading_l
	}registr_r;

static const uint8_t data_tx[] = {NDOF_MODE, AMG_MODE};

/** Reception buffer */
static int8_t data_rx[7] = { 0 };
	
static uint8_t register_bno055[] = {TEMP, EUL_Pitch_MSB, \
	EUL_Pitch_LSB, EUL_Roll_MSB, EUL_Roll_LSB, EUL_Heading_MSB,\
	EUL_Heading_LSB, GYR_DATA_X_LSB, ACC_DATA_X_LSB};

static uint8_t register_bno055_w[] = {OPR_MODE};
/** Global timestamp in milliseconds since start of application */
volatile uint32_t g_ul_ms_ticks = 0;

/**
 *  \brief Handler for System Tick interrupt.
 *
 *  Process System Tick Event
 *  increments the timestamp counter.
 */
void SysTick_Handler(void)
{
	g_ul_ms_ticks++;
}

void TWIHS0_Handler(void)
{
	twihs_get_interrupt_status(TWIHS0);
}

/**
 *  \brief Configure the Console UART.
 *//*
static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
#ifdef CONF_UART_CHAR_LENGTH
		.charlength = CONF_UART_CHAR_LENGTH,
#endif
		.paritytype = CONF_UART_PARITY,
#ifdef CONF_UART_STOP_BITS
		.stopbits = CONF_UART_STOP_BITS,
#endif
	};

	/* Configure console UART. 
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}
*/
/**
 *  \brief Wait for the given number of milliseconds (using the dwTimeStamp
 *         generated by the SAM microcontrollers' system tick).
 *  \param ul_dly_ticks  Delay to wait for, in milliseconds.
 */
static void mdelay(uint32_t ul_dly_ticks)
{
	uint32_t ul_cur_ticks;

	ul_cur_ticks = g_ul_ms_ticks;
	while ((g_ul_ms_ticks - ul_cur_ticks) < ul_dly_ticks);
}


void setup_BNO055(uint8_t mode, uint8_t data, twihs_packet_t point);
void read_BNO055(uint8_t mode, uint8_t data, twihs_packet_t point);

int main(void)
{
	char test_file_name[] = "BNO055_data.txt", buffer_tmp[10] = {0}, buffer_pitch[10] = {0},\
		buffer_roll[10] = {0}, buffer_heading[10] = {0};
	Ctrl_status status;
	FRESULT res;
	FATFS fs;
	FIL file_object;	
	uint32_t i, flag_rx_comp;
	twihs_options_t opt;
	twihs_packet_t packet_tx1, packet_rx1;
	int16_t Temp, Pitch, Roll, Heading;
	static _Bool constanta;
	/* Initialize the SAM system */
	sysclk_init();

	/* Initialize the board */
	board_init();

	/* Turn off LEDs */
	LED_Off(LED0);

    /* Initialize the console UART */
//	configure_console();
	
	/* Initialize SD MMC stack */
	sd_mmc_init();
	
	/* Configure systick for 1 ms */
//	puts("Configure system tick to get 1ms tick period.\r");
	if (SysTick_Config(sysclk_get_cpu_hz() / 1000)) {
		puts("-E- Systick configuration error\r");
		while (1) {
			/* Capture error */
		}
	}

	/* Enable the peripheral clock for TWI */
	pmc_enable_periph_clk(ID_TWIHS0);

	/* Configure the options of TWI driver */
	opt.master_clk = sysclk_get_peripheral_hz();
	opt.speed      = TWIHS_CLK;

	if (twihs_master_init(TWIHS0, &opt) != TWIHS_SUCCESS) {
		puts("-E-\tTWI master initialization failed.\r");
		while (1) {
			/* Capture error */
		}
	}
//	twihs_enable_interrupt(Twihs *p_twihs, uint32_t ul_sources);
	registr_w = opr;
	register_value = ndof;
	
	setup_BNO055((uint8_t)registr_w, (uint8_t)register_value, packet_tx1);

	/* Wait at least 10 ms */
	
	mdelay(WAIT_TIME);
	
	registr_r = temp;
	i = 0;
	flag_rx_comp = 0;
	
	while(1)
	{
		if(registr_r <= heading_l)
		{
			read_BNO055(registr_r, i, packet_rx1);
			registr_r++;
			i++;
		}
		else 
		{

			flag_rx_comp = 1;
			registr_r = temp;
			i = 0;		
		}
		
		if(flag_rx_comp)
		{
			Temp = data_rx[temp];
			Pitch = ((data_rx[pitch_m]<<8)|(data_rx[pitch_l]))/16;
			Roll = ((data_rx[roll_m]<<8)|(data_rx[roll_l]))/16;
			Heading = ((data_rx[heading_m]<<8)|(data_rx[heading_l]))/16;
			flag_rx_comp = 0;
			
			itoa(Temp, buffer_tmp, 10);
			itoa(Pitch, buffer_pitch, 10);
			itoa(Roll, buffer_roll, 10);
			itoa(Heading, buffer_heading, 10);
			
main_end_of_test:
		
			do {
				status = sd_mmc_test_unit_ready(0);
				if (CTRL_FAIL == status) {
					printf("Card install FAIL\n\r");
					printf("Please unplug and re-plug the card.\n\r");
					while (CTRL_NO_PRESENT != sd_mmc_check(0)) {
					}
				}
			} while (CTRL_GOOD != status);

			memset(&fs, 0, sizeof(FATFS));
			res = f_mount(LUN_ID_SD_MMC_0_MEM, &fs);
			if (FR_INVALID_DRIVE == res) {
				printf("[FAIL] res %d\r\n", res);
				constanta = false;
				goto main_end_of_test;
			}
						
			if(!constanta)
			{
				res = f_open(&file_object, (char const *)test_file_name,
				FA_OPEN_ALWAYS | FA_WRITE);
				if (res != FR_OK) {
					printf("[FAIL] res %d\r\n", res);
					constanta = false;
					goto main_end_of_test;
				}
				if (0 == f_printf(&file_object,"Температура |  Рысканье |   Крен | Тангаж | \r\n"))
				{
					f_close(&file_object);
					printf("[FAIL]\r\n");
					constanta = false;
					goto main_end_of_test;
				}
				constanta = true;	
				f_close(&file_object);	
			}
			
			res = f_open(&file_object, (char const *)test_file_name,
			FA_OPEN_ALWAYS | FA_WRITE);
			if (res != FR_OK) {
				printf("[FAIL] res %d\r\n", res);
				constanta = false;
				goto main_end_of_test;
			}
			
			f_lseek (&file_object, file_object.fsize);
				
			if (0 == f_printf(&file_object,"%s           |  %s        |  %s     | %s    | \r\n", \
			buffer_tmp, buffer_heading, buffer_roll, buffer_pitch))
//			if (0 == f_puts(buffer_tmp, &file_object))			 
			{
				f_close(&file_object);
				printf("[FAIL]\r\n");
				constanta = 0;
				goto main_end_of_test;
			}
			f_close(&file_object);	
		}
	}


	/* Match */
	puts("Data comparison:\tMatched!\r");
	LED_On(LED0);
	while (1) {
	}
}


void setup_BNO055(uint8_t mode, uint8_t data, twihs_packet_t point)
{
	
//	if(mode );
	point.chip        = ADDRESS_BNO055;
	point.addr[0]     = register_bno055_w[mode]; //>> 8;
	//	packet_tx.addr[1]     = EEPROM_MEM_ADDR;
	point.addr_length = 0x01;
	point.buffer      = &data_tx[data];
	point.length      = 0x01;
	
	/* Send test pattern to EEPROM */
	if (twihs_master_write(TWIHS0, &point) != TWIHS_SUCCESS) {
		puts("-E-\tTWI master write packet failed.\r");
		while (1) {
			/* Capture error */
		}
	}
}


void read_BNO055(uint8_t mode, uint8_t data, twihs_packet_t point_rx)
{
	
	//	if(mode );
	point_rx.chip        = ADDRESS_BNO055;
	point_rx.addr[0]     = register_bno055[mode];
	//	packet_rx.addr[1]     = packet_tx.addr[1];
	point_rx.addr_length = 0x01;
	point_rx.buffer      = &data_rx[data];
	point_rx.length      = 0x01;
	
	/* Send test pattern to EEPROM */
	if (twihs_master_read(TWIHS0, &point_rx) != TWIHS_SUCCESS) {
		puts("-E-\tTWI master write packet failed.\r");
		while (1) {
			/* Capture error */
		}
	}
}


/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond
