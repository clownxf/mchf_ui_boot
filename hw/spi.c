/************************************************************************************
**                                                                                 **
**                             mcHF Pro QRP Transceiver                            **
**                         Krassi Atanassov - M0NKA 2012-2018                      **
**                            mail: djchrismarc@gmail.com                          **
**                                 twitter: @M0NKA_                                **
**---------------------------------------------------------------------------------**
**                                                                                 **
**  File name:                                                                     **
**  Description:                                                                   **
**  Last Modified:                                                                 **
**  Licence:																	   **
**  			The mcHF project is released for radio amateurs experimentation,   **
**  			non-commercial use only! All source files under GPL-3.0, unless    **
**  			third party drivers specifies otherwise. Thank you!         	   **
**  																			   **
************************************************************************************/

#include "main.h"

#include "spi.h"
#include "gpio.h"
#include "misc.h"

//unsigned char InOutBuffer[MAX_SPI_TRANSFER_SIZE];

// Local IRQ (DSP CS)
static void spi_init_irq_pin(void)
{
	GPIO_InitTypeDef  	GPIO_InitStruct;

	GPIO_InitStruct.Pin 		= ((uint16_t)0x0010U);		// pin 4
	GPIO_InitStruct.Mode 		= ((uint32_t)0x00000000U); 	// input
	//GPIO_InitStruct.Pull 		= ((uint32_t)0x00000000U);
	GPIO_InitStruct.Speed 		= ((uint32_t)0x00000002U);	// high
	gpio_init(GPIOD, &GPIO_InitStruct);
}

#if 0
// Local CS (DSP IRQ)
static void spi_init_cs_pin(void)
{
	GPIO_InitTypeDef  	GPIO_InitStruct;

	GPIO_InitStruct.Pin 		= ((uint16_t)0x0020U);		// pin 5
	GPIO_InitStruct.Mode 		= ((uint32_t)0x00000001U); 	// output
	//GPIO_InitStruct.Pull 		= ((uint32_t)0x00000000U);
	GPIO_InitStruct.Speed 		= ((uint32_t)0x00000002U);	// high
	gpio_init(GPIOD, &GPIO_InitStruct);

	// High by default
	GPIOD->BSRRL = 0x0020U;
}
#endif

// SPI Clock pin(input)
static void spi_init_csk_pin(void)
{
	GPIO_InitTypeDef  	GPIO_InitStruct;

	GPIO_InitStruct.Pin 		= ((uint16_t)0x0200U);		// pin 9
	GPIO_InitStruct.Mode 		= ((uint32_t)0x00000000U); 	// input
	//GPIO_InitStruct.Pull 		= ((uint32_t)0x00000000U);
	GPIO_InitStruct.Speed 		= ((uint32_t)0x00000002U);	// high
	gpio_init(GPIOA, &GPIO_InitStruct);
}

// SPI MOSI pin(input)
static void spi_init_mosi_pin(void)
{
	GPIO_InitTypeDef  	GPIO_InitStruct;

	GPIO_InitStruct.Pin 		= ((uint16_t)0x0002U);		// pin 1
	GPIO_InitStruct.Mode 		= ((uint32_t)0x00000000U); 	// input
	//GPIO_InitStruct.Pull 		= ((uint32_t)0x00000000U);
	GPIO_InitStruct.Speed 		= ((uint32_t)0x00000002U);	// high
	gpio_init(GPIOC, &GPIO_InitStruct);
}

// SPI MISO pin (output)
static void spi_init_miso_pin(void)
{
	GPIO_InitTypeDef  	GPIO_InitStruct;

	GPIO_InitStruct.Pin 		= ((uint16_t)0x0004U);		// pin 2
	GPIO_InitStruct.Mode 		= ((uint32_t)0x00000001U); 	// output
	//GPIO_InitStruct.Pull 		= ((uint32_t)0x00000000U);
	GPIO_InitStruct.Speed 		= ((uint32_t)0x00000002U);	// high
	gpio_init(GPIOC, &GPIO_InitStruct);

	// High by default
	GPIOC->BSRRL = 0x0004U;
}

// Return local IRQ (DSP CS) state
char spi_get_irq_state(void)
{
	if((GPIOD->IDR & 0x0010U) != 0x0010U)
		return 0;
	else
		return 1;
}

//*----------------------------------------------------------------------------
//* Function Name       : spi_receive_block
//* Object              :
//* Input Parameters    :
//* Output Parameters   :
//*----------------------------------------------------------------------------
char spi_receive_block(unsigned char *data, unsigned long size)
{
	unsigned long i,j,timeout;
	unsigned char state;

	// Valid ptr and size ?
	if((size == 0) || (data == (void *)0))
		return 0;

	// Wait(long) DSP CS low
	timeout 	= 0xFFFFFF;
	state		= spi_get_irq_state();
	while(state)
	{
		timeout--;
		if(timeout == 0)
			return 0;

		misc_cpu_dependent_delay(0x7F);

		state = spi_get_irq_state();
	}

	// Receive block
	for(i = 0; i < size; i++)
	{
		data[i] = 0;

		for (j = 0; j < 8; j++)
		{
			// Wait SCK high
			/*timeout 	= 0xFFFFFF;
			state		= (GPIOA->IDR & 0x0200U);
			while(state != 0x0200U)
			{
				timeout--;
				if(timeout == 0)
					return 0;

				misc_cpu_dependent_delay(0x7F);

				state = (GPIOA->IDR & 0x0200U);
			}*/
			while((GPIOA->IDR & 0x0200U) != 0x0200U);

			// Sample MOSI line
			if ((GPIOC->IDR & 0x0002U) == 0x0002U)
				data[i] |= 1 << j;

			// Wait SCK low
			/*timeout 	= 0xFFFFFF;
			state		= (GPIOA->IDR & 0x0200U);
			while(state == 0x0200U)
			{
				timeout--;
				if(timeout == 0)
					return 0;

				misc_cpu_dependent_delay(0x7F);

				state = (GPIOA->IDR & 0x0200U);
			}*/
			while((GPIOA->IDR & 0x0200U) == 0x0200U);
		}
	}

	// Block received
	return 1;
}

//*----------------------------------------------------------------------------
//* Function Name       : spi_transmit_block
//* Object              :
//* Input Parameters    :
//* Output Parameters   :
//*----------------------------------------------------------------------------
void spi_transmit_block(unsigned char *data, unsigned short size)
{
	unsigned long i,j,timeout;
	unsigned char state;

	// Wait(long) DSP CS low
	timeout 	= 0xFFFFFF;
	state		= spi_get_irq_state();
	while(state)
	{
		timeout--;
		if(timeout == 0)
			return;

		misc_cpu_dependent_delay(0x7F);

		state = spi_get_irq_state();
	}
	//while(spi_get_irq_state());

	// Send the transfer buffer
	for (i = 0; i < size; i++)
	{
		// Send one byte
		for (j = 0; j < 8; j++)
		{
			while((GPIOA->IDR & 0x0200U) != 0x0200U);

			if ((data[i] & 1) == 0)
				GPIOC->BSRRH = 0x0004U;
			else
				GPIOC->BSRRL = 0x0004U;

			data[i] = data[i] >> 1;

			while((GPIOA->IDR & 0x0200U) == 0x0200U);
		}
	}
}

void spi_gpio_init(void)
{
	spi_init_irq_pin();
	//spi_init_cs_pin();
	spi_init_csk_pin();
	spi_init_mosi_pin();
	spi_init_miso_pin();
}

