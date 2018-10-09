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

#ifndef __MISC_H
#define __MISC_H

#define KEYLED_XLAT_PIN		((uint16_t)0x0800U)
#define KEYLED_XLAT_PORT	GPIOI
#define KEYLED_BLANK_PIN	((uint16_t)0x0100U)
#define KEYLED_BLANK_PORT	GPIOI

#define KEYLED_SCK_PIN		((uint16_t)0x0020U)
#define KEYLED_SCK_PORT		GPIOA
#define KEYLED_MOSI_PIN		((uint16_t)0x0020U)
#define KEYLED_MOSI_PORT	GPIOB

void misc_clear_buffer(uchar *data, ushort size);
void misc_copy(uchar *out, uchar *in,ushort size);

void misc_init_led(void);
void misc_blink_led(void);
void misc_jump_to_firmware(void);
void misc_cpu_dependent_delay(unsigned long delay);
char misc_check_if_need_to_enter_bootloader_mode(void);

void keypad_led_driver_init(void);
void blink_all(void);

#endif
