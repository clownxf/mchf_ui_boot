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

#ifndef __SPI_H
#define __SPI_H

//#define MAX_SPI_TRANSFER_SIZE	500

void spi_gpio_init(void);

char spi_get_irq_state(void);
char spi_receive_block(unsigned char *data, unsigned long size);
void spi_transmit_block(unsigned char *data, unsigned short size);

#endif
