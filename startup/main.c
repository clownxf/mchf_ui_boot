/************************************************************************************
**                                                                                 **
**                             mcHF Pro QRP Transceiver                            **
**                         Krassi Atanassov - M0NKA 2013-2018                      **
**                              djchrismarc@gmail.com                              **
**                                      @M0NKA_                                    **
**---------------------------------------------------------------------------------**
**                                                                                 **
**  File name:                                                                     **
**  Description:                                                                   **
**  Last Modified:                                                                 **
**  Licence:		For radio amateurs experimentation, non-commercial use only!   **
************************************************************************************/

#include "main.h"

// hardware
#include "gpio.h"
#include "spi.h"
#include "flash.h"
#include "misc.h"

char 			dev_string[] = DEVICE_STRING;
unsigned char 	fat_buff[600];

const unsigned char erase_pass[10] = {
					0xDB, 0x6A, 0x07, 0xF1, 0x0C, 0x02, 0x10, 0x46, 0x98, 0x47
				};

// Process DSP loader commands
void cmd_handler(void)
{
	unsigned char 	buff[300];
	unsigned short 	cmd,i;

next_cmd:

	// Wait for command on SPI
	if(!spi_receive_block(buff,2))
		goto next_cmd;

	cmd = (buff[0] << 8) | buff[1];

	switch(cmd)
	{
		// ----------------------------------------------------
		// Read info
		case 0x07FF:
		{
			misc_clear_buffer(buff,32);

			// Signature
			buff[0] = 0x73;
			buff[1] = 0xF2;

			// Build version
			buff[2] = APPL_MAJOR;
			buff[3] = APPL_MINOR;
			buff[4] = APPL_RELEASE;
			buff[5] = APPL_BUILD;

			// Device string
			misc_copy(buff + 6,(uchar *)dev_string,20);

			// Add other info..
			// ..

			spi_transmit_block(buff,32);
			break;
		}

		// ----------------------------------------------------
		// Erase flash
		case 0x617F:
		{
			uchar erase_on = 0;

			// Get erase pass
			if(spi_receive_block(buff,12))
			{
				// Check password
				for(i = 0; i < 10; i++)
				{
					if(buff[i + 2] != erase_pass[i])
						goto next_cmd;
				}
				erase_on = 1;
			}

			// Erase only on correct pass from DSP bootloader
			if(erase_on)
			{
				// The code stalls here, for a long time!
				flash_unlock();
				flash_erase_firmware();
				flash_lock();
			}

			break;
		}

		// ----------------------------------------------------
		// Check erase result
		case 0x34A1:
		{
			misc_clear_buffer(buff,8);

			// Signal to the Windows utility that the handler
			// is no longer stalled
			buff[0] = 0x22;
			buff[1] = 0x34;
			buff[2] = 0x7B;
			buff[3] = 0xC3;

			spi_transmit_block(buff,8);
			break;
		}

		// ----------------------------------------------------
		// Write flash block
		case 0x214E:
		{
			ulong addr;

			// Get address
			if(spi_receive_block(buff,6))
			{
				// Get args from buffer
				addr  = buff[2] << 24;
				addr |= buff[3] << 16;
				addr |= buff[4] <<  8;
				addr |= buff[5] <<  0;

				// Get block
				if(spi_receive_block(fat_buff,514))
				{
					GPIOD->BSRRL = ((uint16_t)0x0080U);

					flash_unlock();
					flash_write_block((fat_buff + 2),addr);
					flash_lock();
				}
			}

			break;
		}

		// ----------------------------------------------------
		// System reset
		case 0x4466:
		{
			NVIC_SystemReset();
			for(;;)
			{
				__asm(".hword 0x46C0");
			}
			break;
		}

		default:
			break;
	}

	goto next_cmd;
}

int main(void)
{
	// GPIO clocks
	gpio_clocks_on();

	// Prevent keypad LEDs going mad
	keypad_led_driver_init();

	// LED pin init
	misc_init_led();

	// SPI pins init
	spi_gpio_init();

	// Bootloader mode or normal firmware execution
	if(misc_check_if_need_to_enter_bootloader_mode())
	{
		// Pass execution to handler
		// never return
		cmd_handler();
	}

	// Jump to main firmware
	misc_jump_to_firmware();
}

