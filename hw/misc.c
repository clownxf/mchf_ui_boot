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

#include "misc.h"
#include "gpio.h"
#include "spi.h"

uchar btn_id = 0;
ushort pwmbuffer[2*24*1];

void misc_clear_buffer(uchar *data, ushort size)
{
	ushort i;

	for(i = 0; i < size; i++)
		*data++ = 0;
}

void misc_copy(uchar *out, uchar *in,ushort size)
{
	ushort i;

	for(i = 0; i < size; i++)
		*out++ = *in++;
}

void misc_init_lcd_backlight(void)
{
	GPIO_InitTypeDef  	GPIO_InitStruct;

	GPIO_InitStruct.Pin 		= GPIO_PIN_9;
	GPIO_InitStruct.Mode 		= GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull 		= GPIO_NOPULL;
	GPIO_InitStruct.Speed 		= GPIO_SPEED_FREQ_VERY_HIGH;
	gpio_init(GPIOF, &GPIO_InitStruct);

	// LCD Backlight off (active high)
	GPIOF->BSRRH = GPIO_PIN_9;
}

void misc_init_led(void)
{
	GPIO_InitTypeDef  	GPIO_InitStruct;

	GPIO_InitStruct.Pin 		= ((uint16_t)0x0080U);
	GPIO_InitStruct.Mode 		= ((uint32_t)0x00000001U);
	GPIO_InitStruct.Pull 		= ((uint32_t)0x00000000U);
	GPIO_InitStruct.Speed 		= ((uint32_t)0x00000000U);
	gpio_init(GPIOD, &GPIO_InitStruct);

	// LED off
	GPIOD->BSRRH = ((uint16_t)0x0080U);
}

// Blink and stall code
void misc_blink_led(void)
{
	// Blink
	while(1)
	{
		GPIOD->BSRRL = ((uint16_t)0x0080U);
		misc_cpu_dependent_delay(0x7FFFF);

		GPIOD->BSRRH = ((uint16_t)0x0080U);
		misc_cpu_dependent_delay(0x7FFFF);
	}
}

static void keypad_led_mini_delay(void)
{
	uchar i;

	for(i = 0; i < 200; i++)
		__asm(".word 0x46004600");
}

static void keypad_led_shift(void)
{
	KEYLED_XLAT_PORT->BSRRH = KEYLED_XLAT_PIN;
	keypad_led_mini_delay();

	  // 24 channels per TLC5974
	  for (short c=24*1 - 1; c >= 0 ; c--)
	  {
	    // 12 bits per channel, send MSB first
	    for (char b=11; b>=0; b--)
	    {
	    	// Clock low
	    	KEYLED_SCK_PORT->BSRRH = KEYLED_SCK_PIN;
	    	keypad_led_mini_delay();

	    	// Next data bit
	    	if (pwmbuffer[c] & (1 << b))
	    		KEYLED_MOSI_PORT->BSRRL = KEYLED_MOSI_PIN;
	    	else
	    		KEYLED_MOSI_PORT->BSRRH = KEYLED_MOSI_PIN;

	    	// Clock high
	    	KEYLED_SCK_PORT->BSRRL = KEYLED_SCK_PIN;
	    	keypad_led_mini_delay();
	    }
	  }

	  // Clock it out on the Latch pin
	  KEYLED_XLAT_PORT->BSRRH = KEYLED_XLAT_PIN;
	  keypad_led_mini_delay();

	  KEYLED_XLAT_PORT->BSRRL = KEYLED_XLAT_PIN;
	  keypad_led_mini_delay();

	  KEYLED_XLAT_PORT->BSRRH = KEYLED_XLAT_PIN;
	  keypad_led_mini_delay();
}

// Doesn't work!!!
void blink_all(void)
{
	uchar i;
	// -------------------------------
		//if(uc_keep_flag)
		//	pwmbuffer[0] = 0;
		//else
		//	pwmbuffer[0] = 64;

		for(i = 0; i < 24; i++)
			pwmbuffer[i] = 0;

		pwmbuffer[btn_id] = 64;

		btn_id++;
		if(btn_id > 23)
			btn_id = 0;

		keypad_led_shift();
		// --------------------------------
}


void misc_jump_to_firmware(void)
{
	__asm("ldr r0,=0x08020000");	// ptr to firmware vector table
	__asm("ldr r0,[r0,#4]");		// second entry is reset vector addr
	__asm("bx  r0");				// call directly
}

void misc_cpu_dependent_delay(unsigned long delay)
{
	unsigned long i;

	for(i = 0; i < delay; i++)
		__asm(".hword 0x46C0");
}

char misc_check_if_need_to_enter_bootloader_mode(void)
{
	// Wait for the DSP to boot up,
	// much slower than the UI CPU. This delay
	// would probably need to be fine tuned
	misc_cpu_dependent_delay(0x1FFFFF);

	// If both lines(CLK and MOSI) are low
	if(((GPIOA->IDR & 0x0200U) != 0x0200U) &&			// CLK
	   ((GPIOC->IDR & 0x0002U) != 0x0002U)				// MOSI
	  )
	{
		// Wait for DSP bootloader to enter loop (maybe timeout here ?)
		while(spi_get_irq_state());

		// Yes, bootloader mode requested
		return 1;
	}

	return 0;
}

void keypad_led_driver_init(void)
{
	GPIO_InitTypeDef  gpio_init_structure;

	// SCK - PA5 (ToDo: GPIO for start, finally, use SPI HW)
	gpio_init_structure.Pin 	= KEYLED_SCK_PIN;
	gpio_init_structure.Mode 	= GPIO_MODE_OUTPUT_PP;
	gpio_init_structure.Pull 	= ((uint32_t)0x00000001U);
	gpio_init_structure.Speed 	= ((uint32_t)0x00000002U) ;
	gpio_init(KEYLED_SCK_PORT, &gpio_init_structure);

	// MOSI - PB5 (ToDo: GPIO for start, finally, use SPI HW)
	gpio_init_structure.Pin 	= KEYLED_MOSI_PIN;
	gpio_init_structure.Mode 	= GPIO_MODE_OUTPUT_PP;
	gpio_init_structure.Pull 	= ((uint32_t)0x00000001U);
	gpio_init_structure.Speed 	= ((uint32_t)0x00000002U) ;
	gpio_init(KEYLED_MOSI_PORT, &gpio_init_structure);

	// XLAT - PI11
	gpio_init_structure.Pin 	= KEYLED_XLAT_PIN;
	gpio_init_structure.Mode 	= GPIO_MODE_OUTPUT_PP;
	gpio_init_structure.Pull 	= ((uint32_t)0x00000001U);
	gpio_init_structure.Speed 	= ((uint32_t)0x00000002U) ;
	gpio_init(KEYLED_XLAT_PORT, &gpio_init_structure);

	// BLANK - PI8
	gpio_init_structure.Pin 	= KEYLED_BLANK_PIN;
	gpio_init_structure.Mode 	= GPIO_MODE_OUTPUT_PP;
	gpio_init_structure.Pull 	= ((uint32_t)0x00000001U);
	gpio_init_structure.Speed 	= ((uint32_t)0x00000002U) ;
	gpio_init(KEYLED_BLANK_PORT, &gpio_init_structure);

	// BLANK high - all off
	KEYLED_BLANK_PORT->BSRRL = KEYLED_BLANK_PIN;

	// XLAT low
	KEYLED_XLAT_PORT->BSRRH = KEYLED_XLAT_PIN;

	misc_clear_buffer((uchar *)pwmbuffer,(2*24*1)*2);
}
