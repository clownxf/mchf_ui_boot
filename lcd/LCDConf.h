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
**  Licence:                                                                       **
**          The mcHF project is released for radio amateurs experimentation,       **
**          non-commercial use only. All source files under GPL-3.0, unless        **
**          third party drivers specifies otherwise. Thank you!                    **
************************************************************************************/
  
#include "main.h"
#include "GUI.h"
#include "GUIDRV_Lin.h"

#ifndef LCDCONF_H
#define LCDCONF_H

//#undef  LCD_SWAP_XY
//#undef  LCD_MIRROR_Y

//#define LCD_SWAP_XY  	1
//#define LCD_MIRROR_Y	1

#define XSIZE_PHYS 		800
#define YSIZE_PHYS 		480

#define NUM_BUFFERS  	1 /* Number of multiple buffers to be used */
#define NUM_VSCREENS 	1 /* Number of virtual screens to be used */

#define BK_COLOR GUI_DARKBLUE

#undef  GUI_NUM_LAYERS
#define GUI_NUM_LAYERS 	1

#define COLOR_CONVERSION_0 			GUICC_16
#define DISPLAY_DRIVER_0   			GUIDRV_LIN_OXY_8

#define LCD_LAYER0_FRAME_BUFFER  ((int)0x24000000 + (EMWIN_MEMORY*1024))

#define  HSYNC            ((uint16_t)48)   	/* Horizontal synchronization */
#define  HBP              ((uint16_t)88)   	/* Horizontal back porch      */
#define  HFP              ((uint16_t)40)   	/* Horizontal front porch     */
#define  VSYNC            ((uint16_t)3)  	/* Vertical synchronization   */
#define  VBP              ((uint16_t)32)    /* Vertical back porch        */
#define  VFP              ((uint16_t)13)    /* Vertical front porch       */

#if (GUI_NUM_LAYERS > 1)
  #define COLOR_CONVERSION_1 GUICC_M1555I
  #define DISPLAY_DRIVER_1   GUIDRV_LIN_16

#endif

#ifndef   XSIZE_PHYS
  #error Physical X size of display is not defined!
#endif
#ifndef   YSIZE_PHYS
  #error Physical Y size of display is not defined!
#endif
#ifndef   NUM_VSCREENS
  #define NUM_VSCREENS 1
#else
  #if (NUM_VSCREENS <= 0)
    #error At least one screeen needs to be defined!
  #endif
#endif
#if (NUM_VSCREENS > 1) && (NUM_BUFFERS > 1)
  #error Virtual screens and multiple buffers are not allowed!
#endif

typedef struct
{
  int32_t      address;          
  __IO int32_t      pending_buffer;   
  int32_t      buffer_index;     
  int32_t      xSize;            
  int32_t      ySize;            
  int32_t      BytesPerPixel;
  LCD_API_COLOR_CONV   *pColorConvAPI;
}
LCD_LayerPropTypedef;

#endif /* LCDCONF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
