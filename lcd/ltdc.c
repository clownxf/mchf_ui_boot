/************************************************************************************
**                                                                                 **
**                             mcHF Pro QRP Transceiver                            **
**                         Krassi Atanassov - M0NKA 2012-2019                      **
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

#include "ltdc.h"

#if 0
void HAL_DMA2D_MspDeInit(DMA2D_HandleTypeDef *hdma2d)
{
  /* Enable DMA2D reset state */
  __HAL_RCC_DMA2D_FORCE_RESET();

  /* Release DMA2D from reset state */
  __HAL_RCC_DMA2D_RELEASE_RESET();
}
#endif

uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);   /* only values 0..7 are used          */
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(__NVIC_PRIO_BITS)) ? (uint32_t)(__NVIC_PRIO_BITS) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(__NVIC_PRIO_BITS)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(__NVIC_PRIO_BITS));

  return (
           ((PreemptPriority & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL)) << SubPriorityBits) |
           ((SubPriority     & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL)))
         );
}

uint32_t NVIC_GetPriorityGrouping(void)
{
  return ((uint32_t)((SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) >> SCB_AIRCR_PRIGROUP_Pos));
}

void NVIC_SetPriority(int32_t IRQn, uint32_t priority)
{
  if ((int32_t)(IRQn) < 0)
  {
    SCB->SHPR[(((uint32_t)(int32_t)IRQn) & 0xFUL)-4UL] = (uint8_t)((priority << (8U - __NVIC_PRIO_BITS)) & (uint32_t)0xFFUL);
  }
  else
  {
    NVIC->IP[((uint32_t)(int32_t)IRQn)]                = (uint8_t)((priority << (8U - __NVIC_PRIO_BITS)) & (uint32_t)0xFFUL);
  }
}

void HAL_NVIC_SetPriority(int32_t IRQn, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t prioritygroup = 0x00;

  /* Check the parameters */
  //assert_param(IS_NVIC_SUB_PRIORITY(SubPriority));
  //assert_param(IS_NVIC_PREEMPTION_PRIORITY(PreemptPriority));

  prioritygroup = NVIC_GetPriorityGrouping();

  NVIC_SetPriority(IRQn, NVIC_EncodePriority(prioritygroup, PreemptPriority, SubPriority));
}

static void LTDC_SetConfig(LTDC_HandleTypeDef *hltdc, LTDC_LayerCfgTypeDef *pLayerCfg, uint32_t LayerIdx)
{
  uint32_t tmp = 0;
  uint32_t tmp1 = 0;
  uint32_t tmp2 = 0;

  /* Configure the horizontal start and stop position */
  tmp = ((pLayerCfg->WindowX1 + ((hltdc->Instance->BPCR & LTDC_BPCR_AHBP) >> 16)) << 16);
  LTDC_LAYER(hltdc, LayerIdx)->WHPCR &= ~(LTDC_LxWHPCR_WHSTPOS | LTDC_LxWHPCR_WHSPPOS);
  LTDC_LAYER(hltdc, LayerIdx)->WHPCR = ((pLayerCfg->WindowX0 + ((hltdc->Instance->BPCR & LTDC_BPCR_AHBP) >> 16) + 1) | tmp);

  /* Configure the vertical start and stop position */
  tmp = ((pLayerCfg->WindowY1 + (hltdc->Instance->BPCR & LTDC_BPCR_AVBP)) << 16);
  LTDC_LAYER(hltdc, LayerIdx)->WVPCR &= ~(LTDC_LxWVPCR_WVSTPOS | LTDC_LxWVPCR_WVSPPOS);
  LTDC_LAYER(hltdc, LayerIdx)->WVPCR  = ((pLayerCfg->WindowY0 + (hltdc->Instance->BPCR & LTDC_BPCR_AVBP) + 1) | tmp);

  /* Specifies the pixel format */
  LTDC_LAYER(hltdc, LayerIdx)->PFCR &= ~(LTDC_LxPFCR_PF);
  LTDC_LAYER(hltdc, LayerIdx)->PFCR = (pLayerCfg->PixelFormat);

  /* Configure the default color values */
  tmp = ((uint32_t)(pLayerCfg->Backcolor.Green) << 8);
  tmp1 = ((uint32_t)(pLayerCfg->Backcolor.Red) << 16);
  tmp2 = (pLayerCfg->Alpha0 << 24);
  LTDC_LAYER(hltdc, LayerIdx)->DCCR &= ~(LTDC_LxDCCR_DCBLUE | LTDC_LxDCCR_DCGREEN | LTDC_LxDCCR_DCRED | LTDC_LxDCCR_DCALPHA);
  LTDC_LAYER(hltdc, LayerIdx)->DCCR = (pLayerCfg->Backcolor.Blue | tmp | tmp1 | tmp2);

  /* Specifies the constant alpha value */
  LTDC_LAYER(hltdc, LayerIdx)->CACR &= ~(LTDC_LxCACR_CONSTA);
  LTDC_LAYER(hltdc, LayerIdx)->CACR = (pLayerCfg->Alpha);

  /* Specifies the blending factors */
  LTDC_LAYER(hltdc, LayerIdx)->BFCR &= ~(LTDC_LxBFCR_BF2 | LTDC_LxBFCR_BF1);
  LTDC_LAYER(hltdc, LayerIdx)->BFCR = (pLayerCfg->BlendingFactor1 | pLayerCfg->BlendingFactor2);

  /* Configure the color frame buffer start address */
  LTDC_LAYER(hltdc, LayerIdx)->CFBAR &= ~(LTDC_LxCFBAR_CFBADD);
  LTDC_LAYER(hltdc, LayerIdx)->CFBAR = (pLayerCfg->FBStartAdress);

  if(pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_ARGB8888)
  {
    tmp = 4;
  }
  else if (pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_RGB888)
  {
    tmp = 3;
  }
  else if((pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_ARGB4444) || \
    (pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_RGB565)   || \
      (pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_ARGB1555) || \
        (pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_AL88))
  {
    tmp = 2;
  }
  else
  {
    tmp = 1;
  }

  /* Configure the color frame buffer pitch in byte */
  LTDC_LAYER(hltdc, LayerIdx)->CFBLR  &= ~(LTDC_LxCFBLR_CFBLL | LTDC_LxCFBLR_CFBP);
  LTDC_LAYER(hltdc, LayerIdx)->CFBLR  = (((pLayerCfg->ImageWidth * tmp) << 16) | (((pLayerCfg->WindowX1 - pLayerCfg->WindowX0) * tmp)  + 7));

  /* Configure the frame buffer line number */
  LTDC_LAYER(hltdc, LayerIdx)->CFBLNR  &= ~(LTDC_LxCFBLNR_CFBLNBR);
  LTDC_LAYER(hltdc, LayerIdx)->CFBLNR  = (pLayerCfg->ImageHeight);

  /* Enable LTDC_Layer by setting LEN bit */
  LTDC_LAYER(hltdc, LayerIdx)->CR |= (uint32_t)LTDC_LxCR_LEN;
}

static void LTDC_MspInit(LTDC_HandleTypeDef *hltdc)
{
	GPIO_InitTypeDef gpio_init_structure;

	/* Enable the LTDC and DMA2D clocks */
	//__HAL_RCC_LTDC_CLK_ENABLE();
	//__HAL_RCC_DMA2D_CLK_ENABLE();

	  // -------------------------------------------------------------------------------------------------
	  // Multiplexer allocated pins, important - LTDC could use AF9,AF12 or AF14 !!!!
	  // -------------------------------------------------------------------------------------------------
	  gpio_init_structure.Mode      = GPIO_MODE_AF_PP;
	  gpio_init_structure.Pull      = GPIO_NOPULL;
	  gpio_init_structure.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
	  // -------------------------------------------------------------------------------------------------
	  #ifndef PCB_REV_26JULY18
	  // GPIOA LTDC AF12 & AF14
  	  //
  	  //								LCD_R1(AF14)	LCD_B5(AF14)	LCD_VSYNC(AF14)	LCD_G2(AF14)
  	  //								LCD_R6(AF14)	LCD_R4(AF14)	LCD_R5(AF14)
	  gpio_init_structure.Pin       = 	GPIO_PIN_2 | 	GPIO_PIN_3  | 	GPIO_PIN_4  |	GPIO_PIN_6 |\
			  	  	  	  				GPIO_PIN_8 | 	GPIO_PIN_11 | 	GPIO_PIN_12;
	  #endif
	  #ifdef PCB_REV_26JULY18
	  // GPIOA LTDC AF12 & AF14
	  //
	  //								LCD_R2(AF14)	LCD_R1(AF14)	LCD_B5(AF14)	LCD_VSYNC(AF14)
	  //								LCD_G2(AF14)	LCD_R6(AF14)	LCD_R4(AF14)	LCD_R5(AF14)
	  gpio_init_structure.Pin       = 	GPIO_PIN_1 |	GPIO_PIN_2 | 	GPIO_PIN_3  | 	GPIO_PIN_4  |\
			  	  	  	  	  	  	  	GPIO_PIN_6 |	GPIO_PIN_8 | 	GPIO_PIN_11 | 	GPIO_PIN_12;
	  #endif
	  gpio_init_structure.Alternate = 	GPIO_AF14_LTDC;
	  gpio_init(GPIOA, &gpio_init_structure);
	  //								LCD_B4(AF12)
	  gpio_init_structure.Pin       = 	 GPIO_PIN_10;
	  gpio_init_structure.Alternate = 	GPIO_AF12_LTDC;
	  gpio_init(GPIOA, &gpio_init_structure);
	  // -------------------------------------------------------------------------------------------------
	  // GPIOB LTDC AF9 & AF14
	  //								LCD_R3(AF9)
	  gpio_init_structure.Pin       = 	GPIO_PIN_0;
	  gpio_init_structure.Alternate = 	GPIO_AF9_LTDC;
	  gpio_init(GPIOB, &gpio_init_structure);
	  //								LCD_B6(AF14)	LCD_B7(AF14)	LCD_G4(AF14)	LCD_G5(AF14)
	  gpio_init_structure.Pin       = 	GPIO_PIN_8  | 	GPIO_PIN_9  | 	GPIO_PIN_10 | 	GPIO_PIN_11;
	  gpio_init_structure.Alternate = 	GPIO_AF14_LTDC;
	  gpio_init(GPIOB, &gpio_init_structure);
	  // -------------------------------------------------------------------------------------------------
	  #ifndef PCB_REV_26JULY18
	  // GPIOC LTDC AF14
	  //								LCD_HSYNC(AF14)	LCD_G6(AF14)	LCD_R2(AF14)
	  gpio_init_structure.Pin       = 	GPIO_PIN_6  | 	GPIO_PIN_7  | 	GPIO_PIN_10;
	  #endif
	  #ifdef PCB_REV_26JULY18
	  // GPIOC LTDC AF14
	  //								LCD_HSYNC(AF14)	LCD_G6(AF14)
	  gpio_init_structure.Pin       = 	GPIO_PIN_6  | 	GPIO_PIN_7;
	  #endif
	  gpio_init_structure.Alternate =	GPIO_AF14_LTDC;
	  gpio_init(GPIOC, &gpio_init_structure);
	  // -------------------------------------------------------------------------------------------------
	  // GPIOD LTDC AF14
	  //								LCD_G7(AF14)	LCD_B2(AF14)
	  gpio_init_structure.Pin       = 	GPIO_PIN_3  | 	GPIO_PIN_6;
	  gpio_init_structure.Alternate = 	GPIO_AF14_LTDC;
	  gpio_init(GPIOD, &gpio_init_structure);
	  // -------------------------------------------------------------------------------------------------
	  // GPIOE LTDC AF14
	  //								LCD_B0(AF14)	LCD_G0(AF14)	LCD_G1(AF14)
	  gpio_init_structure.Pin       = 	GPIO_PIN_4  | 	GPIO_PIN_5  | 	GPIO_PIN_6;
	  gpio_init_structure.Alternate = 	GPIO_AF14_LTDC;
	  gpio_init(GPIOE, &gpio_init_structure);
	  // -------------------------------------------------------------------------------------------------
	  // GPIOF LTDC AF14
	  //								LCD_DE(AF14)
	  gpio_init_structure.Pin       = 	GPIO_PIN_10;
	  gpio_init_structure.Alternate = 	GPIO_AF14_LTDC;
	  gpio_init(GPIOF, &gpio_init_structure);
	  // -------------------------------------------------------------------------------------------------
	  // GPIOG LTDC AF9 & AF14
	  //								LCD_G3(AF9)
	  gpio_init_structure.Pin       = 	GPIO_PIN_10;
	  gpio_init_structure.Alternate = 	GPIO_AF9_LTDC;
	  gpio_init(GPIOG, &gpio_init_structure);
	  //								LCD_R7(AF14)	LCD_CLK(AF14)	LCD_B3(AF14)	LCD_B1(AF14)
	  //								LCD_R0(AF14)
	  gpio_init_structure.Pin       = 	GPIO_PIN_6  | 	GPIO_PIN_7 |	GPIO_PIN_11 | 	GPIO_PIN_12 |\
			  	  	  	  	  	  	  	GPIO_PIN_13;
	  gpio_init_structure.Alternate = 	GPIO_AF14_LTDC;
	  gpio_init(GPIOG, &gpio_init_structure);
	  // -----------------------------------------------------------------------------------------
	  // GPIO pins
	  // -----------------------------------------------------------------------------------------
	  // LCD_DISP GPIO configuration
	  gpio_init_structure.Pin       = GPIO_PIN_7;
	  gpio_init_structure.Mode      = GPIO_MODE_OUTPUT_PP;
	  gpio_init(GPIOF, &gpio_init_structure);
	  // -------------------------------------------------------------------------------------------------
	  // LCD_BL_CTRL GPIO configuration
	  gpio_init_structure.Pin       = GPIO_PIN_9;
	  gpio_init_structure.Mode      = GPIO_MODE_OUTPUT_PP;
	  gpio_init(GPIOF, &gpio_init_structure);
	  // -------------------------------------------------------------------------------------------------
	  // -------------------------------------------------------------------------------------------------

	  // Assert display enable LCD_DISP pin - off
	  GPIOF->BSRRH = GPIO_PIN_7;

	  // Assert backlight LCD_BL_CTRL pin - off
	  GPIOF->BSRRH = GPIO_PIN_9;

	  // Need that ?
	  /* Set LTDC Interrupt to the lowest priority */
	  HAL_NVIC_SetPriority(88, 0xE, 0);
	  /* Enable LTDC Interrupt */
	  //HAL_NVIC_EnableIRQ(LTDC_IRQn);
	  NVIC->ISER[(((uint32_t)(int32_t)88) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)88) & 0x1FUL));
}

/**
  * @brief LTDC MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  * @param hltdc: LTDC handle pointer
  * @retval None
  */
static void LTDC_MspDeInit(LTDC_HandleTypeDef *hltdc)
{
#if 0
	/* Reset peripherals */
  /* Enable LTDC reset state */
  __HAL_RCC_LTDC_FORCE_RESET();

  /* Release LTDC from reset state */
  __HAL_RCC_LTDC_RELEASE_RESET();
#endif
}

/**
  * @brief  Configure the PLL3 VCI,VCO ranges, multiplication and division factors and enable it
  * @param  pll3: Pointer to an RCC_PLL3InitTypeDef structure that
  *         contains the configuration parameters  as well as VCI, VCO clock ranges.
  * @note   PLL3 is temporary disable to apply new parameters
  *
  * @retval HAL status
  */
static void RCCEx_PLL3_Config(RCC_PLL3InitTypeDef *pll3)
{
#if 1
  uint32_t tickstart = 0;
  //HAL_StatusTypeDef status = HAL_OK;

  //assert_param(IS_RCC_PLL3M_VALUE(pll3->PLL3M));
  //assert_param(IS_RCC_PLL3N_VALUE(pll3->PLL3N));
  //assert_param(IS_RCC_PLL3P_VALUE(pll3->PLL3P));
  //assert_param(IS_RCC_PLL3R_VALUE(pll3->PLL3R));
  //assert_param(IS_RCC_PLL3Q_VALUE(pll3->PLL3Q));

  /* Check that PLL3 OSC clock source is already set */
  if(__HAL_RCC_GET_PLL_OSCSOURCE() == RCC_PLLSOURCE_NONE)
  {
	  // ToDo: Handle this error !!!!
      return;
  }
  else
  {
    /* Disable  PLL3. */
    __HAL_RCC_PLL3_DISABLE();

#if 0
    /* Get Start Tick*/
    tickstart = HAL_GetTick();

    /* Wait till PLL3 is ready */
    while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLL3RDY) != RESET)
    {
      if((int32_t) (HAL_GetTick() - tickstart ) > PLL3_TIMEOUT_VALUE)
      {
        return HAL_TIMEOUT;
      }
    }
#endif
    // ToDo: Use normal delay with timeout ....

    /* Configure the PLL3  multiplication and division factors. */
    __HAL_RCC_PLL3_CONFIG(pll3->PLL3M,
                          pll3->PLL3N,
                          pll3->PLL3P,
                          pll3->PLL3Q,
                          pll3->PLL3R);

    /* Select PLL3 input reference frequency range: VCI */
   __HAL_RCC_PLL3_VCIRANGE(pll3->PLL3RGE) ;

    /* Select PLL3 output frequency range : VCO */
   __HAL_RCC_PLL3_VCORANGE(pll3->PLL3VCOSEL) ;

    /* Enable  PLL3. */
    __HAL_RCC_PLL3_ENABLE();

#if 0
    /* Get Start Tick*/
    tickstart = HAL_GetTick();

    /* Wait till PLL3 is ready */
    while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLL3RDY) == RESET)
    {
      if((int32_t) (HAL_GetTick() - tickstart ) > PLL3_TIMEOUT_VALUE)
      {
        return HAL_TIMEOUT;
      }
    }
#endif
    // ToDo: Use normal delay with timeout ....

  }


  //return status;
#endif
}

void LTDC_DeInit(LTDC_HandleTypeDef *hltdc)
{
  /* DeInit the low level hardware */
  LTDC_MspDeInit(hltdc);

  /* Initialize the error code */
  hltdc->ErrorCode = HAL_LTDC_ERROR_NONE;

  /* Initialize the LTDC state*/
  hltdc->State = HAL_LTDC_STATE_RESET;

  /* Release Lock */
  //__HAL_UNLOCK(hltdc);
}

void LTDC_InitPLL(RCC_PeriphCLKInitTypeDef  *PeriphClkInit)
{
	__HAL_RCC_PLL3CLKOUT_ENABLE(RCC_PLL3_DIVR);
	RCCEx_PLL3_Config(&(PeriphClkInit->PLL3));
}

/**
  * @brief  Initialize the LTDC according to the specified parameters in the LTDC_InitTypeDef.
  * @param  hltdc  pointer to a LTDC_HandleTypeDef structure that contains
  *                the configuration information for the LTDC.
  * @retval HAL status
  */
void LTDC_Init(LTDC_HandleTypeDef *hltdc)
{
  uint32_t tmp = 0, tmp1 = 0;

  /* Check the LTDC peripheral state */
  if(hltdc == NULL)
  {
    return;
  }

  /* Check function parameters */
  //assert_param(IS_LTDC_ALL_INSTANCE(hltdc->Instance));
  //assert_param(IS_LTDC_HSYNC(hltdc->Init.HorizontalSync));
  //assert_param(IS_LTDC_VSYNC(hltdc->Init.VerticalSync));
  //assert_param(IS_LTDC_AHBP(hltdc->Init.AccumulatedHBP));
  //assert_param(IS_LTDC_AVBP(hltdc->Init.AccumulatedVBP));
  //assert_param(IS_LTDC_AAH(hltdc->Init.AccumulatedActiveH));
  //assert_param(IS_LTDC_AAW(hltdc->Init.AccumulatedActiveW));
  //assert_param(IS_LTDC_TOTALH(hltdc->Init.TotalHeigh));
  //assert_param(IS_LTDC_TOTALW(hltdc->Init.TotalWidth));
  //assert_param(IS_LTDC_HSPOL(hltdc->Init.HSPolarity));
  //assert_param(IS_LTDC_VSPOL(hltdc->Init.VSPolarity));
  //assert_param(IS_LTDC_DEPOL(hltdc->Init.DEPolarity));
  //assert_param(IS_LTDC_PCPOL(hltdc->Init.PCPolarity));


  if(hltdc->State == HAL_LTDC_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    hltdc->Lock = HAL_UNLOCKED;
    /* Init the low level hardware */
    LTDC_MspInit(hltdc);
  }

  /* Change LTDC peripheral state */
  hltdc->State = HAL_LTDC_STATE_BUSY;

  /* Configure the HS, VS, DE and PC polarity */
  hltdc->Instance->GCR &= ~(LTDC_GCR_HSPOL | LTDC_GCR_VSPOL | LTDC_GCR_DEPOL | LTDC_GCR_PCPOL);
  hltdc->Instance->GCR |=  (uint32_t)(hltdc->Init.HSPolarity | hltdc->Init.VSPolarity | \
  hltdc->Init.DEPolarity | hltdc->Init.PCPolarity);

  /* Set Synchronization size */
  hltdc->Instance->SSCR &= ~(LTDC_SSCR_VSH | LTDC_SSCR_HSW);
  tmp = (hltdc->Init.HorizontalSync << 16);
  hltdc->Instance->SSCR |= (tmp | hltdc->Init.VerticalSync);

  /* Set Accumulated Back porch */
  hltdc->Instance->BPCR &= ~(LTDC_BPCR_AVBP | LTDC_BPCR_AHBP);
  tmp = (hltdc->Init.AccumulatedHBP << 16);
  hltdc->Instance->BPCR |= (tmp | hltdc->Init.AccumulatedVBP);

  /* Set Accumulated Active Width */
  hltdc->Instance->AWCR &= ~(LTDC_AWCR_AAH | LTDC_AWCR_AAW);
  tmp = (hltdc->Init.AccumulatedActiveW << 16);
  hltdc->Instance->AWCR |= (tmp | hltdc->Init.AccumulatedActiveH);

  /* Set Total Width */
  hltdc->Instance->TWCR &= ~(LTDC_TWCR_TOTALH | LTDC_TWCR_TOTALW);
  tmp = (hltdc->Init.TotalWidth << 16);
  hltdc->Instance->TWCR |= (tmp | hltdc->Init.TotalHeigh);

  /* Set the background color value */
  tmp = ((uint32_t)(hltdc->Init.Backcolor.Green) << 8);
  tmp1 = ((uint32_t)(hltdc->Init.Backcolor.Red) << 16);
  hltdc->Instance->BCCR &= ~(LTDC_BCCR_BCBLUE | LTDC_BCCR_BCGREEN | LTDC_BCCR_BCRED);
  hltdc->Instance->BCCR |= (tmp1 | tmp | hltdc->Init.Backcolor.Blue);

  /* Enable the Transfer Error and FIFO underrun interrupts */
  __HAL_LTDC_ENABLE_IT(hltdc, LTDC_IT_TE | LTDC_IT_FU);

  /* Enable LTDC by setting LTDCEN bit */
  __HAL_LTDC_ENABLE(hltdc);

  /* Initialize the error code */
  hltdc->ErrorCode = HAL_LTDC_ERROR_NONE;

  /* Initialize the LTDC state*/
  hltdc->State = HAL_LTDC_STATE_READY;
}

void LTDC_SetAddress(LTDC_HandleTypeDef *hltdc, uint32_t Address, uint32_t LayerIdx)
{
  LTDC_LayerCfgTypeDef *pLayerCfg;

  /* Check the parameters */
  //assert_param(IS_LTDC_LAYER(LayerIdx));

  /* Process locked */
 // __HAL_LOCK(hltdc);

  /* Change LTDC peripheral state */
  hltdc->State = HAL_LTDC_STATE_BUSY;

  /* Get layer configuration from handle structure */
  pLayerCfg = &hltdc->LayerCfg[LayerIdx];

  /* Reconfigure the Address */
  pLayerCfg->FBStartAdress = Address;

  /* Set LTDC parameters */
  LTDC_SetConfig(hltdc, pLayerCfg, LayerIdx);

  /* Set the Immediate Reload type */
  hltdc->Instance->SRCR = LTDC_SRCR_IMR;

  /* Change the LTDC state*/
  hltdc->State = HAL_LTDC_STATE_READY;

  /* Process unlocked */
 // __HAL_UNLOCK(hltdc);
}

void LTDC_ConfigCLUT(LTDC_HandleTypeDef *hltdc, uint32_t *pCLUT, uint32_t CLUTSize, uint32_t LayerIdx)
{
  uint32_t tmp = 0;
  uint32_t counter = 0;
  uint32_t pcounter = 0;

  /* Check the parameters */
  //assert_param(IS_LTDC_LAYER(LayerIdx));

  /* Process locked */
  //__HAL_LOCK(hltdc);

  /* Change LTDC peripheral state */
  hltdc->State = HAL_LTDC_STATE_BUSY;

  for(counter = 0; (counter < CLUTSize); counter++)
  {
    if(hltdc->LayerCfg[LayerIdx].PixelFormat == LTDC_PIXEL_FORMAT_AL44)
    {
      tmp  = (((counter + 16*counter) << 24) | ((uint32_t)(*pCLUT) & 0xFF) | ((uint32_t)(*pCLUT) & 0xFF00) | ((uint32_t)(*pCLUT) & 0xFF0000));
    }
    else
    {
      tmp  = ((counter << 24) | ((uint32_t)(*pCLUT) & 0xFF) | ((uint32_t)(*pCLUT) & 0xFF00) | ((uint32_t)(*pCLUT) & 0xFF0000));
    }
    pcounter = (uint32_t)pCLUT + sizeof(*pCLUT);
    pCLUT = (uint32_t *)pcounter;

    /* Specifies the C-LUT address and RGB value */
    LTDC_LAYER(hltdc, LayerIdx)->CLUTWR  = tmp;
  }

  /* Change the LTDC state*/
  hltdc->State = HAL_LTDC_STATE_READY;

  /* Process unlocked */
  //__HAL_UNLOCK(hltdc);
}

void LTDC_SetWindowPosition(LTDC_HandleTypeDef *hltdc, uint32_t X0, uint32_t Y0, uint32_t LayerIdx)
{
  LTDC_LayerCfgTypeDef *pLayerCfg;

  /* Check the parameters */
  //assert_param(IS_LTDC_LAYER(LayerIdx));
  //assert_param(IS_LTDC_CFBLL(X0));
  //assert_param(IS_LTDC_CFBLNBR(Y0));

  /* Process locked */
  //__HAL_LOCK(hltdc);

  /* Change LTDC peripheral state */
  hltdc->State = HAL_LTDC_STATE_BUSY;

  /* Get layer configuration from handle structure */
  pLayerCfg = &hltdc->LayerCfg[LayerIdx];

  /* update horizontal start/stop */
  pLayerCfg->WindowX0 = X0;
  pLayerCfg->WindowX1 = X0 + pLayerCfg->ImageWidth;

  /* update vertical start/stop */
  pLayerCfg->WindowY0 = Y0;
  pLayerCfg->WindowY1 = Y0 + pLayerCfg->ImageHeight;

  /* Set LTDC parameters */
  LTDC_SetConfig(hltdc, pLayerCfg, LayerIdx);

  /* Set the Immediate Reload type */
  hltdc->Instance->SRCR = LTDC_SRCR_IMR;

  /* Change the LTDC state*/
  hltdc->State = HAL_LTDC_STATE_READY;

  /* Process unlocked */
  //__HAL_UNLOCK(hltdc);
}

void LTDC_EnableColorKeying(LTDC_HandleTypeDef *hltdc, uint32_t LayerIdx)
{
  /* Check the parameters */
  //assert_param(IS_LTDC_LAYER(LayerIdx));

  /* Process locked */
  //__HAL_LOCK(hltdc);

  /* Change LTDC peripheral state */
  hltdc->State = HAL_LTDC_STATE_BUSY;

  /* Enable LTDC color keying by setting COLKEN bit */
  LTDC_LAYER(hltdc, LayerIdx)->CR |= (uint32_t)LTDC_LxCR_COLKEN;

  /* Set the Immediate Reload type */
  hltdc->Instance->SRCR = LTDC_SRCR_IMR;

  /* Change the LTDC state*/
  hltdc->State = HAL_LTDC_STATE_READY;

  /* Process unlocked */
  //__HAL_UNLOCK(hltdc);
}

void LTDC_DisableColorKeying(LTDC_HandleTypeDef *hltdc, uint32_t LayerIdx)
{
  /* Check the parameters */
  //assert_param(IS_LTDC_LAYER(LayerIdx));

  /* Process locked */
  //__HAL_LOCK(hltdc);

  /* Change LTDC peripheral state */
  hltdc->State = HAL_LTDC_STATE_BUSY;

  /* Disable LTDC color keying by setting COLKEN bit */
  LTDC_LAYER(hltdc, LayerIdx)->CR &= ~(uint32_t)LTDC_LxCR_COLKEN;

  /* Set the Immediate Reload type */
  hltdc->Instance->SRCR = LTDC_SRCR_IMR;

  /* Change the LTDC state*/
  hltdc->State = HAL_LTDC_STATE_READY;

  /* Process unlocked */
  //__HAL_UNLOCK(hltdc);
}

void LTDC_ConfigColorKeying(LTDC_HandleTypeDef *hltdc, uint32_t RGBValue, uint32_t LayerIdx)
{
  /* Check the parameters */
  //assert_param(IS_LTDC_LAYER(LayerIdx));

  /* Process locked */
  //__HAL_LOCK(hltdc);

  /* Change LTDC peripheral state */
  hltdc->State = HAL_LTDC_STATE_BUSY;

  /* Configure the default color values */
  LTDC_LAYER(hltdc, LayerIdx)->CKCR &=  ~(LTDC_LxCKCR_CKBLUE | LTDC_LxCKCR_CKGREEN | LTDC_LxCKCR_CKRED);
  LTDC_LAYER(hltdc, LayerIdx)->CKCR  = RGBValue;

  /* Set the Immediate Reload type */
  hltdc->Instance->SRCR = LTDC_SRCR_IMR;

  /* Change the LTDC state*/
  hltdc->State = HAL_LTDC_STATE_READY;

  /* Process unlocked */
  //__HAL_UNLOCK(hltdc);
}

void LTDC_ProgramLineEvent(LTDC_HandleTypeDef *hltdc, uint32_t Line)
{
  /* Check the parameters */
  //assert_param(IS_LTDC_LIPOS(Line));

  /* Process locked */
  //__HAL_LOCK(hltdc);

  /* Change LTDC peripheral state */
  hltdc->State = HAL_LTDC_STATE_BUSY;

  /* Disable the Line interrupt */
  __HAL_LTDC_DISABLE_IT(hltdc, LTDC_IT_LI);

  /* Set the Line Interrupt position */
  LTDC->LIPCR = (uint32_t)Line;

  /* Enable the Line interrupt */
  __HAL_LTDC_ENABLE_IT(hltdc, LTDC_IT_LI);

  /* Change the LTDC state*/
  hltdc->State = HAL_LTDC_STATE_READY;

  /* Process unlocked */
  //__HAL_UNLOCK(hltdc);
}

void LTDC_ConfigLayer(LTDC_HandleTypeDef *hltdc, LTDC_LayerCfgTypeDef *pLayerCfg, uint32_t LayerIdx)
{
  /* Check the parameters */
  //assert_param(IS_LTDC_LAYER(LayerIdx));
  //assert_param(IS_LTDC_HCONFIGST(pLayerCfg->WindowX0));
  //assert_param(IS_LTDC_HCONFIGSP(pLayerCfg->WindowX1));
  //assert_param(IS_LTDC_VCONFIGST(pLayerCfg->WindowY0));
  //assert_param(IS_LTDC_VCONFIGSP(pLayerCfg->WindowY1));
  //assert_param(IS_LTDC_PIXEL_FORMAT(pLayerCfg->PixelFormat));
  //assert_param(IS_LTDC_ALPHA(pLayerCfg->Alpha));
  //assert_param(IS_LTDC_ALPHA(pLayerCfg->Alpha0));
  //assert_param(IS_LTDC_BLENDING_FACTOR1(pLayerCfg->BlendingFactor1));
  //assert_param(IS_LTDC_BLENDING_FACTOR2(pLayerCfg->BlendingFactor2));
  //assert_param(IS_LTDC_CFBLL(pLayerCfg->ImageWidth));
  //assert_param(IS_LTDC_CFBLNBR(pLayerCfg->ImageHeight));

  /* Process locked */
  //__HAL_LOCK(hltdc);

  /* Change LTDC peripheral state */
  hltdc->State = HAL_LTDC_STATE_BUSY;

  /* Copy new layer configuration into handle structure */
  hltdc->LayerCfg[LayerIdx] = *pLayerCfg;

  /* Configure the LTDC Layer */
  LTDC_SetConfig(hltdc, pLayerCfg, LayerIdx);

  /* Set the Immediate Reload type */
  hltdc->Instance->SRCR = LTDC_SRCR_IMR;

  /* Initialize the LTDC state*/
  hltdc->State  = HAL_LTDC_STATE_READY;

  /* Process unlocked */
  //__HAL_UNLOCK(hltdc);
}

void LTDC_EnableCLUT(LTDC_HandleTypeDef *hltdc, uint32_t LayerIdx)
{
  /* Check the parameters */
  //assert_param(IS_LTDC_LAYER(LayerIdx));

  /* Process locked */
  //__HAL_LOCK(hltdc);

  /* Change LTDC peripheral state */
  hltdc->State = HAL_LTDC_STATE_BUSY;

  /* Enable LTDC color lookup table by setting CLUTEN bit */
  LTDC_LAYER(hltdc, LayerIdx)->CR |= (uint32_t)LTDC_LxCR_CLUTEN;

  /* Set the Immediate Reload type */
  hltdc->Instance->SRCR = LTDC_SRCR_IMR;

  /* Change the LTDC state*/
  hltdc->State = HAL_LTDC_STATE_READY;

  /* Process unlocked */
  //__HAL_UNLOCK(hltdc);
}

void DMA2D_Init(DMA2D_HandleTypeDef *hdma2d)
{
  /* Check the DMA2D peripheral state */
  if(hdma2d == NULL)
     return;

  /* Check the parameters */
  //assert_param(IS_DMA2D_ALL_INSTANCE(hdma2d->Instance));
  //assert_param(IS_DMA2D_MODE(hdma2d->Init.Mode));
  //assert_param(IS_DMA2D_CMODE(hdma2d->Init.ColorMode));
  //assert_param(IS_DMA2D_OFFSET(hdma2d->Init.OutputOffset));

  if(hdma2d->State == HAL_DMA2D_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    hdma2d->Lock = HAL_UNLOCKED;
  }

  /* Change DMA2D peripheral state */
  hdma2d->State = HAL_DMA2D_STATE_BUSY;

  /* DMA2D CR register configuration -------------------------------------------*/
  MODIFY_REG(hdma2d->Instance->CR, DMA2D_CR_MODE, hdma2d->Init.Mode);

  /* DMA2D OPFCCR register configuration ---------------------------------------*/
  MODIFY_REG(hdma2d->Instance->OPFCCR, DMA2D_OPFCCR_CM, hdma2d->Init.ColorMode);

  /* DMA2D OOR register configuration ------------------------------------------*/
  MODIFY_REG(hdma2d->Instance->OOR, DMA2D_OOR_LO, hdma2d->Init.OutputOffset);

  /* DMA2D OPFCCR AI fields setting (Output Alpha Inversion)*/
  MODIFY_REG(hdma2d->Instance->OPFCCR, DMA2D_OPFCCR_AI, (hdma2d->Init.AlphaInverted << DMA2D_POSITION_OPFCCR_AI));

  MODIFY_REG(hdma2d->Instance->OPFCCR, DMA2D_OPFCCR_RBS,(hdma2d->Init.RedBlueSwap << DMA2D_POSITION_OPFCCR_RBS));

  /* Update error code */
  hdma2d->ErrorCode = HAL_DMA2D_ERROR_NONE;

  /* Initialize the DMA2D state*/
  hdma2d->State  = HAL_DMA2D_STATE_READY;
}

void LTDC_SetAlpha(LTDC_HandleTypeDef *hltdc, uint32_t Alpha, uint32_t LayerIdx)
{
  LTDC_LayerCfgTypeDef *pLayerCfg;

  /* Check the parameters */
  //assert_param(IS_LTDC_ALPHA(Alpha));
  //assert_param(IS_LTDC_LAYER(LayerIdx));

  /* Process locked */
  //__HAL_LOCK(hltdc);

  /* Change LTDC peripheral state */
  hltdc->State = HAL_LTDC_STATE_BUSY;

  /* Get layer configuration from handle structure */
  pLayerCfg = &hltdc->LayerCfg[LayerIdx];

  /* Reconfigure the Alpha value */
  pLayerCfg->Alpha = Alpha;

  /* Set LTDC parameters */
  LTDC_SetConfig(hltdc, pLayerCfg, LayerIdx);

  /* Set the Immediate Reload type */
  hltdc->Instance->SRCR = LTDC_SRCR_IMR;

  /* Change the LTDC state*/
  hltdc->State = HAL_LTDC_STATE_READY;

  /* Process unlocked */
  //__HAL_UNLOCK(hltdc);
}

void LTDC_IRQHandlerA(LTDC_HandleTypeDef *hltdc)
{
#if 1
  /* Transfer Error Interrupt management ***************************************/
  if(__HAL_LTDC_GET_FLAG(hltdc, LTDC_FLAG_TE) != RESET)
  {
    if(__HAL_LTDC_GET_IT_SOURCE(hltdc, LTDC_IT_TE) != RESET)
    {
      /* Disable the transfer Error interrupt */
      __HAL_LTDC_DISABLE_IT(hltdc, LTDC_IT_TE);

      /* Clear the transfer error flag */
      __HAL_LTDC_CLEAR_FLAG(hltdc, LTDC_FLAG_TE);

      /* Update error code */
      hltdc->ErrorCode |= HAL_LTDC_ERROR_TE;

      /* Change LTDC state */
      hltdc->State = HAL_LTDC_STATE_ERROR;

      /* Process unlocked */
      //__HAL_UNLOCK(hltdc);

      /* Transfer error Callback */
      //HAL_LTDC_ErrorCallback(hltdc);
    }
  }
  /* FIFO underrun Interrupt management ***************************************/
  if(__HAL_LTDC_GET_FLAG(hltdc, LTDC_FLAG_FU) != RESET)
  {
    if(__HAL_LTDC_GET_IT_SOURCE(hltdc, LTDC_IT_FU) != RESET)
    {
      /* Disable the FIFO underrun interrupt */
      __HAL_LTDC_DISABLE_IT(hltdc, LTDC_IT_FU);

      /* Clear the FIFO underrun flag */
      __HAL_LTDC_CLEAR_FLAG(hltdc, LTDC_FLAG_FU);

      /* Update error code */
      hltdc->ErrorCode |= HAL_LTDC_ERROR_FU;

      /* Change LTDC state */
      hltdc->State = HAL_LTDC_STATE_ERROR;

      /* Process unlocked */
      //__HAL_UNLOCK(hltdc);

      /* Transfer error Callback */
      //HAL_LTDC_ErrorCallback(hltdc);
    }
  }
  /* Line Interrupt management ************************************************/
  if(__HAL_LTDC_GET_FLAG(hltdc, LTDC_FLAG_LI) != RESET)
  {
    if(__HAL_LTDC_GET_IT_SOURCE(hltdc, LTDC_IT_LI) != RESET)
    {
      /* Disable the Line interrupt */
      __HAL_LTDC_DISABLE_IT(hltdc, LTDC_IT_LI);

      /* Clear the Line interrupt flag */
      __HAL_LTDC_CLEAR_FLAG(hltdc, LTDC_FLAG_LI);

      /* Change LTDC state */
      hltdc->State = HAL_LTDC_STATE_READY;

      /* Process unlocked */
      //__HAL_UNLOCK(hltdc);

      /* Line interrupt Callback */
      //HAL_LTDC_LineEventCallback(hltdc);
    }
  }
  /* Register reload Interrupt management ***************************************/
  if(__HAL_LTDC_GET_FLAG(hltdc, LTDC_FLAG_RR) != RESET)
  {
    if(__HAL_LTDC_GET_IT_SOURCE(hltdc, LTDC_IT_RR) != RESET)
    {
      /* Disable the register reload interrupt */
      __HAL_LTDC_DISABLE_IT(hltdc, LTDC_IT_RR);

      /* Clear the register reload flag */
      __HAL_LTDC_CLEAR_FLAG(hltdc, LTDC_FLAG_RR);

      /* Change LTDC state */
      hltdc->State = HAL_LTDC_STATE_READY;

      /* Process unlocked */
      //__HAL_UNLOCK(hltdc);

      /* Register reload interrupt Callback */
      //HAL_LTDC_ReloadEventCallback(hltdc);
    }
  }
#endif
}

