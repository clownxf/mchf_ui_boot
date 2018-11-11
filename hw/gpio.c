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

#include "gpio.h"

void gpio_init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init)
{
  uint32_t position = 0x00;
  uint32_t ioposition = 0x00;
  uint32_t iocurrent = 0x00;
  uint32_t temp = 0x00;

  /* Configure the port pins */
  for(position = 0; position < 16; position++)
  {
    /* Get the IO position */
    ioposition = ((uint32_t)0x01) << position;
    /* Get the current IO position */
    iocurrent = (uint32_t)(GPIO_Init->Pin) & ioposition;

    if(iocurrent == ioposition)
    {
      /*--------------------- GPIO Mode Configuration ------------------------*/
      /* In case of Alternate function mode selection */
      if((GPIO_Init->Mode == GPIO_MODE_AF_PP) || (GPIO_Init->Mode == GPIO_MODE_AF_OD))
      {
        /* Check the Alternate function parameter */
        //assert_param(IS_GPIO_AF(GPIO_Init->Alternate));

        /* Configure Alternate function mapped with the current IO */
        temp = GPIOx->AFR[position >> 3];
        temp &= ~((uint32_t)0xF << ((uint32_t)(position & (uint32_t)0x07) * 4)) ;
        temp |= ((uint32_t)(GPIO_Init->Alternate) << (((uint32_t)position & (uint32_t)0x07) * 4));
        GPIOx->AFR[position >> 3] = temp;
      }

      /* Configure IO Direction mode (Input, Output, Alternate or Analog) */
      temp = GPIOx->MODER;
      temp &= ~(GPIO_MODER_MODER0 << (position * 2));
      temp |= ((GPIO_Init->Mode & GPIO_MODE) << (position * 2));
      GPIOx->MODER = temp;

      /* In case of Output or Alternate function mode selection */
      if((GPIO_Init->Mode == GPIO_MODE_OUTPUT_PP) || (GPIO_Init->Mode == GPIO_MODE_AF_PP) ||
         (GPIO_Init->Mode == GPIO_MODE_OUTPUT_OD) || (GPIO_Init->Mode == GPIO_MODE_AF_OD))
      {
        /* Check the Speed parameter */
        //assert_param(IS_GPIO_SPEED(GPIO_Init->Speed));
        /* Configure the IO Speed */
        temp = GPIOx->OSPEEDR;
        temp &= ~(GPIO_OSPEEDER_OSPEEDR0 << (position * 2));
        temp |= (GPIO_Init->Speed << (position * 2));
        GPIOx->OSPEEDR = temp;

        /* Configure the IO Output Type */
        temp = GPIOx->OTYPER;
        temp &= ~(GPIO_OTYPER_OT_0 << position) ;
        temp |= (((GPIO_Init->Mode & GPIO_OUTPUT_TYPE) >> 4) << position);
        GPIOx->OTYPER = temp;
      }

      /* Activate the Pull-up or Pull down resistor for the current IO */
      temp = GPIOx->PUPDR;
      temp &= ~(GPIO_PUPDR_PUPDR0 << (position * 2));
      temp |= ((GPIO_Init->Pull) << (position * 2));
      GPIOx->PUPDR = temp;
    }
  }
}

unsigned long gpio_clocks_on(void)
{
	unsigned long readback;

	// PORTA clock on
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN;
	__asm(".hword 0x46C0");
	readback = RCC->AHB4ENR & RCC_AHB4ENR_GPIOAEN;

	// PORTB clock on
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN;
	__asm(".hword 0x46C0");
	readback = RCC->AHB4ENR & RCC_AHB4ENR_GPIOBEN;

	// PORTC clock on
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOCEN;
	__asm(".hword 0x46C0");
	readback = RCC->AHB4ENR & RCC_AHB4ENR_GPIOCEN;

	// PORTD clock on
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIODEN;
	__asm(".hword 0x46C0");
	readback = RCC->AHB4ENR & RCC_AHB4ENR_GPIOCEN;

	// PORTE clock on
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOEEN;
	__asm(".hword 0x46C0");
	readback = RCC->AHB4ENR & RCC_AHB4ENR_GPIOEEN;

	// PORTF clock on
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOFEN;
	__asm(".hword 0x46C0");
	readback = RCC->AHB4ENR & RCC_AHB4ENR_GPIOFEN;

	// PORTG clock on
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOGEN;
	__asm(".hword 0x46C0");
	readback = RCC->AHB4ENR & RCC_AHB4ENR_GPIOGEN;

	// PORTI clock on
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOIEN;
	__asm(".hword 0x46C0");
	readback = RCC->AHB4ENR & RCC_AHB4ENR_GPIOIEN;

	// CRC clock (for LCD lib unlock)
	RCC->AHB4ENR |= RCC_AHB4ENR_CRCEN;
	__asm(".hword 0x46C0");
	readback = RCC->AHB4ENR & RCC_AHB4ENR_CRCEN;

	// SRAM1 clock on
	RCC->AHB2ENR |= RCC_AHB2ENR_D2SRAM1EN;
	__asm(".hword 0x46C0");
	readback = RCC->AHB2ENR & RCC_AHB2ENR_D2SRAM1EN;

	// LTDC clock on
	RCC->APB3ENR |= RCC_APB3ENR_LTDCEN;
	__asm(".hword 0x46C0");
	readback = RCC->APB3ENR & RCC_APB3ENR_LTDCEN;

	// DMA2D clock on
	RCC->APB3ENR |= RCC_AHB3ENR_DMA2DEN;
	__asm(".hword 0x46C0");
	readback = RCC->APB3ENR & RCC_AHB3ENR_DMA2DEN;

	return readback;
}
