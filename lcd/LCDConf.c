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

#include "LCDConf.h"
#include "GUI_Private.h"
#include "ltdc.h"

LTDC_HandleTypeDef            	hltdc;
DMA2D_HandleTypeDef           	hdma2d;
LCD_LayerPropTypedef          	layer_prop[GUI_NUM_LAYERS];
RCC_PeriphCLKInitTypeDef  		periph_clk_init_struct;

static const LCD_API_COLOR_CONV * apColorConvAPI[] = 
{
  COLOR_CONVERSION_0,
#if GUI_NUM_LAYERS > 1
  COLOR_CONVERSION_1,
#endif
};

void LTDC_IRQHandler(void)
{
  LTDC_IRQHandlerA(&hltdc);
}

static inline U32 LCD_LL_GetPixelformat(U32 LayerIndex)
{
  //if (LayerIndex == 0)
  //  return LTDC_PIXEL_FORMAT_RGB565;
  //else
  //  return LTDC_PIXEL_FORMAT_ARGB1555;

  return LTDC_PIXEL_FORMAT_L8;
}

static void DMA2D_CopyBuffer(U32 LayerIndex, void * pSrc, void * pDst, U32 xSize, U32 ySize, U32 OffLineSrc, U32 OffLineDst)
{
  U32 PixelFormat;

  PixelFormat = LCD_LL_GetPixelformat(LayerIndex);
  DMA2D->CR      = 0x00000000UL | (1 << 9);

  /* Set up pointers */
  DMA2D->FGMAR   = (U32)pSrc;
  DMA2D->OMAR    = (U32)pDst;
  DMA2D->FGOR    = OffLineSrc;
  DMA2D->OOR     = OffLineDst;

  /* Set up pixel format */
  DMA2D->FGPFCCR = PixelFormat;

  /*  Set up size */
  DMA2D->NLR     = (U32)(xSize << 16) | (U16)ySize;

  DMA2D->CR     |= DMA2D_CR_START;

  /* Wait until transfer is done */
  while (DMA2D->CR & DMA2D_CR_START)
  {
  }
}

static void DMA2D_FillBuffer(U32 LayerIndex, void * pDst, U32 xSize, U32 ySize, U32 OffLine, U32 ColorIndex)
{
  U32 PixelFormat;

  PixelFormat = LCD_LL_GetPixelformat(LayerIndex);

  /* Set up mode */
  DMA2D->CR      = 0x00030000UL | (1 << 9);
  DMA2D->OCOLR   = ColorIndex;

  /* Set up pointers */
  DMA2D->OMAR    = (U32)pDst;

  /* Set up offsets */
  DMA2D->OOR     = OffLine;

  /* Set up pixel format */
  DMA2D->OPFCCR  = PixelFormat;

  /*  Set up size */
  DMA2D->NLR     = (U32)(xSize << 16) | (U16)ySize;

  DMA2D->CR     |= DMA2D_CR_START;

  /* Wait until transfer is done */
  while (DMA2D->CR & DMA2D_CR_START)
  {
  }
}

static void LCD_LL_Init(void)
{
  // DeInit
  LTDC_DeInit(&hltdc);

  // Set LCD Timings
  hltdc.Init.HorizontalSync 	= (HSYNC - 1);
  hltdc.Init.VerticalSync 		= (VSYNC - 1);
  hltdc.Init.AccumulatedHBP 	= (HSYNC + HBP - 1);
  hltdc.Init.AccumulatedVBP 	= (VSYNC + VBP - 1);
  hltdc.Init.AccumulatedActiveH = (YSIZE_PHYS + VSYNC + VBP - 1);
  hltdc.Init.AccumulatedActiveW = (XSIZE_PHYS + HSYNC + HBP - 1);
  hltdc.Init.TotalHeigh 		= (YSIZE_PHYS + VSYNC + VBP + VFP - 1);
  hltdc.Init.TotalWidth 		= (XSIZE_PHYS + HSYNC + HBP + HFP - 1);

  // background value
  hltdc.Init.Backcolor.Blue 	= 0;
  hltdc.Init.Backcolor.Green 	= 0;
  hltdc.Init.Backcolor.Red 		= 0;

  //  LTDC Clock = 10.625 Mhz (ok)
  periph_clk_init_struct.PLL3.PLL3M = 32;
  periph_clk_init_struct.PLL3.PLL3N = 85;
  periph_clk_init_struct.PLL3.PLL3R = 4;

  periph_clk_init_struct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  periph_clk_init_struct.PLL3.PLL3P = 2;
  periph_clk_init_struct.PLL3.PLL3Q = 2;
  LTDC_InitPLL(&periph_clk_init_struct);

  /* Polarity */
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Instance = LTDC;

  LTDC_Init(&hltdc);
  LTDC_ProgramLineEvent(&hltdc, 0);

  /* Enable dithering */
  //HAL_LTDC_EnableDither(&hltdc);

   /* Configure the DMA2D default mode */
  hdma2d.Init.Mode         = DMA2D_R2M;
  hdma2d.Init.ColorMode    = DMA2D_INPUT_RGB565;
  hdma2d.Init.OutputOffset = 0x0;
  hdma2d.Instance          = DMA2D;

  DMA2D_Init(&hdma2d);

  // LCD on
   GPIOF->BSRRL = GPIO_PIN_7;
   GPIOF->BSRRL = GPIO_PIN_9;
}

static void LCD_LL_LayerInit(U32 LayerIndex)
{
  LTDC_LayerCfgTypeDef             layer_cfg;

  if (LayerIndex < GUI_NUM_LAYERS)
  {
    /* Layer configuration */
    layer_cfg.WindowX0 = 0;
    layer_cfg.WindowX1 = XSIZE_PHYS;
    layer_cfg.WindowY0 = 0;
    layer_cfg.WindowY1 = YSIZE_PHYS;
    layer_cfg.PixelFormat = LCD_LL_GetPixelformat(LayerIndex);
    layer_cfg.FBStartAdress = layer_prop[LayerIndex].address;
    layer_cfg.Alpha = 255;
    layer_cfg.Alpha0 = 0;
    layer_cfg.Backcolor.Blue = 0;
    layer_cfg.Backcolor.Green = 0;
    layer_cfg.Backcolor.Red = 0;
    layer_cfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
    layer_cfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
    layer_cfg.ImageWidth = XSIZE_PHYS;
    layer_cfg.ImageHeight = YSIZE_PHYS;
    LTDC_ConfigLayer(&hltdc, &layer_cfg, LayerIndex);

    /* Enable LUT on demand */
    if (LCD_GetBitsPerPixelEx(LayerIndex) <= 8)
    {
      /* Enable usage of LUT for all modes with <= 8bpp*/
      LTDC_EnableCLUT(&hltdc, LayerIndex);
    }
  }
}

static U32 GetBufferSize(U32 LayerIndex)
{

  U32 BufferSize;

  BufferSize = layer_prop[LayerIndex].xSize * layer_prop[LayerIndex].ySize * layer_prop[LayerIndex].BytesPerPixel;
  return BufferSize;
}

void CUSTOM_DrawBitmap16bpp(int LayerIndex, int x, int y, U16 const * p, int xSize, int ySize, int BytesPerLine)
{
  U32 BufferSize, AddrDst;
  int OffLineSrc, OffLineDst;

  BufferSize = GetBufferSize(LayerIndex);
  AddrDst = layer_prop[LayerIndex].address + BufferSize * layer_prop[LayerIndex].buffer_index + (y * layer_prop[LayerIndex].xSize + x) * layer_prop[LayerIndex].BytesPerPixel;
  OffLineSrc = (BytesPerLine / 2) - xSize;
  OffLineDst = layer_prop[LayerIndex].xSize - xSize;
  DMA2D_CopyBuffer(LayerIndex, (void *)p, (void *)AddrDst, xSize, ySize, OffLineSrc, OffLineDst);
}

static void CUSTOM_CopyBuffer(int LayerIndex, int IndexSrc, int IndexDst)
{
  U32 BufferSize, AddrSrc, AddrDst;

  BufferSize = GetBufferSize(LayerIndex);
  AddrSrc    = layer_prop[LayerIndex].address + BufferSize * IndexSrc;
  AddrDst    = layer_prop[LayerIndex].address + BufferSize * IndexDst;
  DMA2D_CopyBuffer(LayerIndex, (void *)AddrSrc, (void *)AddrDst, layer_prop[LayerIndex].xSize, layer_prop[LayerIndex].ySize, 0, 0);
  layer_prop[LayerIndex].buffer_index = IndexDst;
}

static void CUSTOM_CopyRect(int LayerIndex, int x0, int y0, int x1, int y1, int xSize, int ySize)
{
  U32 AddrSrc, AddrDst;

  AddrSrc = layer_prop[LayerIndex].address + (y0 * layer_prop[LayerIndex].xSize + x0) * layer_prop[LayerIndex].BytesPerPixel;
  AddrDst = layer_prop[LayerIndex].address + (y1 * layer_prop[LayerIndex].xSize + x1) * layer_prop[LayerIndex].BytesPerPixel;
  DMA2D_CopyBuffer(LayerIndex, (void *)AddrSrc, (void *)AddrDst, xSize, ySize, layer_prop[LayerIndex].xSize - xSize, layer_prop[LayerIndex].xSize - xSize);

/*	U32 AddrSrc, AddrDst;
	int l_y0, l_y1;

	// Swapped LCD
	l_y0 = 480 - y0;
	l_y1 = 480 - y1;

	// Calculate source
	AddrSrc = layer_prop[LayerIndex].address + (l_y0 * layer_prop[LayerIndex].xSize + x0) * layer_prop[LayerIndex].BytesPerPixel;

	// Calculate destination
	AddrDst = layer_prop[LayerIndex].address + (l_y1 * layer_prop[LayerIndex].xSize + x1) * layer_prop[LayerIndex].BytesPerPixel;

	// Copy via DMA
	DMA2D_CopyBuffer(	LayerIndex,
						(void *)AddrSrc,
						(void *)AddrDst,
						xSize,
						ySize,
						(layer_prop[LayerIndex].xSize - xSize),
						(layer_prop[LayerIndex].xSize - xSize)
					);
*/
}

static void CUSTOM_FillRect(int LayerIndex, int x0, int y0, int x1, int y1, U32 PixelIndex)
{
  U32 BufferSize, AddrDst;
  int xSize, ySize;

  /* Data Cahce management */
  if (GUI_GetDrawMode() == GUI_DM_XOR)
  {
    LCD_SetDevFunc(LayerIndex, LCD_DEVFUNC_FILLRECT, NULL);
    LCD_FillRect(x0, y0, x1, y1);
    LCD_SetDevFunc(LayerIndex, LCD_DEVFUNC_FILLRECT, (void(*)(void))CUSTOM_FillRect);
  }
  else
  {
    xSize = x1 - x0 + 1;
    ySize = y1 - y0 + 1;
    BufferSize = GetBufferSize(LayerIndex);
	AddrDst = layer_prop[LayerIndex].address + BufferSize * layer_prop[LayerIndex].buffer_index + (y0 * layer_prop[LayerIndex].xSize + x0) * layer_prop[LayerIndex].BytesPerPixel;
    DMA2D_FillBuffer(LayerIndex, (void *)AddrDst, xSize, ySize, layer_prop[LayerIndex].xSize - xSize, PixelIndex);
	}
}

void CUSTOM_DrawBitmap32bpp(int LayerIndex, int x, int y, U8 const * p, int xSize, int ySize, int BytesPerLine)
{
  U32 BufferSize, AddrDst;
  int OffLineSrc, OffLineDst;

  BufferSize = GetBufferSize(LayerIndex);
  AddrDst = layer_prop[LayerIndex].address + BufferSize * layer_prop[LayerIndex].buffer_index + (y * layer_prop[LayerIndex].xSize + x) * layer_prop[LayerIndex].BytesPerPixel;
  OffLineSrc = (BytesPerLine / 4) - xSize;
  OffLineDst = layer_prop[LayerIndex].xSize - xSize;
  DMA2D_CopyBuffer(LayerIndex, (void *)p, (void *)AddrDst, xSize, ySize, OffLineSrc, OffLineDst);
}

void HAL_LTDC_LineEvenCallback(LTDC_HandleTypeDef *hltdc)
{
  U32 Addr;
  U32 layer;

  for (layer = 0; layer < GUI_NUM_LAYERS; layer++)
  {
    if (layer_prop[layer].pending_buffer >= 0) 
    {
      /* Calculate address of buffer to be used  as visible frame buffer */
      Addr = layer_prop[layer].address + \
             layer_prop[layer].xSize * layer_prop[layer].ySize * layer_prop[layer].pending_buffer * layer_prop[layer].BytesPerPixel;
      
      __HAL_LTDC_LAYER(hltdc, layer)->CFBAR = Addr;
     
      __HAL_LTDC_RELOAD_CONFIG(hltdc);
      
      /* Notify STemWin that buffer is used */
      GUI_MULTIBUF_ConfirmEx(layer, layer_prop[layer].pending_buffer);

      /* Clear pending buffer flag of layer */
      layer_prop[layer].pending_buffer = -1;
    }
  }
  
  LTDC_ProgramLineEvent(hltdc, 0);
}

void LCD_X_Config(void) 
{
  U32 i;

  LCD_LL_Init ();
    
  /* At first initialize use of multiple buffers on demand */
#if (NUM_BUFFERS > 1)
    for (i = 0; i < GUI_NUM_LAYERS; i++) 
    {
      GUI_MULTIBUF_ConfigEx(i, NUM_BUFFERS);
    }
#endif

  /* Set display driver and color conversion for 1st layer */
  GUI_DEVICE_CreateAndLink(DISPLAY_DRIVER_0, COLOR_CONVERSION_0, 0, 0);
  
  /* Set size of 1st layer */
  if (LCD_GetSwapXYEx(0)) {
    LCD_SetSizeEx (0, YSIZE_PHYS, XSIZE_PHYS);
    LCD_SetVSizeEx(0, YSIZE_PHYS * NUM_VSCREENS, XSIZE_PHYS);
  } else {
    LCD_SetSizeEx (0, XSIZE_PHYS, YSIZE_PHYS);
    LCD_SetVSizeEx(0, XSIZE_PHYS, YSIZE_PHYS * NUM_VSCREENS);
  }
  #if (GUI_NUM_LAYERS > 1)
    
    /* Set display driver and color conversion for 2nd layer */
    GUI_DEVICE_CreateAndLink(DISPLAY_DRIVER_1, COLOR_CONVERSION_1, 0, 1);
    
    /* Set size of 2nd layer */
    if (LCD_GetSwapXYEx(1)) {
      LCD_SetSizeEx (1, YSIZE_PHYS, XSIZE_PHYS);
      LCD_SetVSizeEx(1, YSIZE_PHYS * NUM_VSCREENS, XSIZE_PHYS);
    } else {
      LCD_SetSizeEx (1, XSIZE_PHYS, YSIZE_PHYS);
      LCD_SetVSizeEx(1, XSIZE_PHYS, YSIZE_PHYS * NUM_VSCREENS);
    }
  #endif
  
    /*Initialise GUI Layer structure */
    layer_prop[0].address = LCD_LAYER0_FRAME_BUFFER;
    
#if (GUI_NUM_LAYERS > 1)    
    layer_prop[1].address = LCD_LAYER1_FRAME_BUFFER; 
#endif
       
   /* Setting up VRam address and custom functions for CopyBuffer-, CopyRect- and FillRect operations */
  for (i = 0; i < GUI_NUM_LAYERS; i++) 
  {

    layer_prop[i].pColorConvAPI = (LCD_API_COLOR_CONV *)apColorConvAPI[i];
     
    layer_prop[i].pending_buffer = -1;

    /* Set VRAM address */
    LCD_SetVRAMAddrEx(i, (void *)(layer_prop[i].address));

    /* Remember color depth for further operations */
    layer_prop[i].BytesPerPixel = LCD_GetBitsPerPixelEx(i) >> 3;

    /* Set custom functions for several operations */
    LCD_SetDevFunc(i, LCD_DEVFUNC_COPYBUFFER, (void(*)(void))CUSTOM_CopyBuffer);
    LCD_SetDevFunc(i, LCD_DEVFUNC_COPYRECT,   (void(*)(void))CUSTOM_CopyRect);
    LCD_SetDevFunc(i, LCD_DEVFUNC_FILLRECT, (void(*)(void))CUSTOM_FillRect);

    /* Set up drawing routine for 32bpp bitmap using DMA2D */
    if (LCD_LL_GetPixelformat(i) == LTDC_PIXEL_FORMAT_ARGB8888) {
     LCD_SetDevFunc(i, LCD_DEVFUNC_DRAWBMP_32BPP, (void(*)(void))CUSTOM_DrawBitmap32bpp);     /* Set up drawing routine for 32bpp bitmap using DMA2D. Makes only sense with ARGB8888 */
    }
  }
}

int LCD_X_DisplayDriver(unsigned LayerIndex, unsigned Cmd, void * pData) 
{
	int r = 0;
	U32 addr;
	int xPos, yPos;
	U32 Color;
    
	switch (Cmd)
	{
		case LCD_X_INITCONTROLLER:
			LCD_LL_LayerInit(LayerIndex);
			break;

		case LCD_X_SETORG:
			addr = layer_prop[LayerIndex].address + ((LCD_X_SETORG_INFO *)pData)->yPos * layer_prop[LayerIndex].xSize * layer_prop[LayerIndex].BytesPerPixel;
			LTDC_SetAddress(&hltdc, addr, LayerIndex);
			break;

		case LCD_X_SHOWBUFFER:
			layer_prop[LayerIndex].pending_buffer = ((LCD_X_SHOWBUFFER_INFO *)pData)->Index;
			break;

		case LCD_X_SETLUTENTRY:
			LTDC_ConfigCLUT(&hltdc, (uint32_t *)&(((LCD_X_SETLUTENTRY_INFO *)pData)->Color), 1, LayerIndex);
			break;

		case LCD_X_ON:
			__HAL_LTDC_ENABLE(&hltdc);
			break;

		case LCD_X_OFF:
			__HAL_LTDC_DISABLE(&hltdc);
			break;
    
		case LCD_X_SETVIS:
		{
			if(((LCD_X_SETVIS_INFO *)pData)->OnOff  == ENABLE )
				__HAL_LTDC_LAYER_ENABLE(&hltdc, LayerIndex);
			else
				__HAL_LTDC_LAYER_DISABLE(&hltdc, LayerIndex);

			__HAL_LTDC_RELOAD_CONFIG(&hltdc);
			break;
		}
    
		case LCD_X_SETPOS:
			LTDC_SetWindowPosition(&hltdc,((LCD_X_SETPOS_INFO *)pData)->xPos,((LCD_X_SETPOS_INFO *)pData)->yPos,LayerIndex);
			break;

		case LCD_X_SETSIZE:
			GUI_GetLayerPosEx(LayerIndex, &xPos, &yPos);
			layer_prop[LayerIndex].xSize = ((LCD_X_SETSIZE_INFO *)pData)->xSize;
			layer_prop[LayerIndex].ySize = ((LCD_X_SETSIZE_INFO *)pData)->ySize;
			LTDC_SetWindowPosition(&hltdc, xPos, yPos, LayerIndex);
			break;

		case LCD_X_SETALPHA:
		    LTDC_SetAlpha(&hltdc, ((LCD_X_SETALPHA_INFO *)pData)->Alpha, LayerIndex);
		    break;

		case LCD_X_SETCHROMAMODE:
		{
			if(((LCD_X_SETCHROMAMODE_INFO *)pData)->ChromaMode != 0)
				LTDC_EnableColorKeying(&hltdc, LayerIndex);
			else
				LTDC_DisableColorKeying(&hltdc, LayerIndex);
			break;
		}

		case LCD_X_SETCHROMA:
			Color = ((((LCD_X_SETCHROMA_INFO *)pData)->ChromaMin & 0xFF0000) >> 16) |\
					(((LCD_X_SETCHROMA_INFO *)pData)->ChromaMin & 0x00FF00) |\
					((((LCD_X_SETCHROMA_INFO *)pData)->ChromaMin & 0x0000FF) << 16);
    
			LTDC_ConfigColorKeying(&hltdc, Color, LayerIndex);
			break;

		default:
			r = -1;
  }

  return r;
}








