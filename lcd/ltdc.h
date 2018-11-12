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

#ifndef __LTDC_H
#define __LTDC_H

#include "gpio.h"

/**
  * @brief LCD-TFT Display Controller
  */

typedef struct
{
  uint32_t      RESERVED0[2];  /*!< Reserved, 0x00-0x04 */
  __IO uint32_t SSCR;          /*!< LTDC Synchronization Size Configuration Register,    Address offset: 0x08 */
  __IO uint32_t BPCR;          /*!< LTDC Back Porch Configuration Register,              Address offset: 0x0C */
  __IO uint32_t AWCR;          /*!< LTDC Active Width Configuration Register,            Address offset: 0x10 */
  __IO uint32_t TWCR;          /*!< LTDC Total Width Configuration Register,             Address offset: 0x14 */
  __IO uint32_t GCR;           /*!< LTDC Global Control Register,                        Address offset: 0x18 */
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x1C-0x20 */
  __IO uint32_t SRCR;          /*!< LTDC Shadow Reload Configuration Register,           Address offset: 0x24 */
  uint32_t      RESERVED2[1];  /*!< Reserved, 0x28 */
  __IO uint32_t BCCR;          /*!< LTDC Background Color Configuration Register,        Address offset: 0x2C */
  uint32_t      RESERVED3[1];  /*!< Reserved, 0x30 */
  __IO uint32_t IER;           /*!< LTDC Interrupt Enable Register,                      Address offset: 0x34 */
  __IO uint32_t ISR;           /*!< LTDC Interrupt Status Register,                      Address offset: 0x38 */
  __IO uint32_t ICR;           /*!< LTDC Interrupt Clear Register,                       Address offset: 0x3C */
  __IO uint32_t LIPCR;         /*!< LTDC Line Interrupt Position Configuration Register, Address offset: 0x40 */
  __IO uint32_t CPSR;          /*!< LTDC Current Position Status Register,               Address offset: 0x44 */
  __IO uint32_t CDSR;         /*!< LTDC Current Display Status Register,                       Address offset: 0x48 */
} LTDC_TypeDef;

/** @defgroup LTDC_Exported_Types LTDC Exported Types
  * @{
  */
#define MAX_LAYER  2U

/**
  * @brief  LTDC color structure definition
  */
typedef struct
{
  uint8_t Blue;                    /*!< Configures the blue value.
                                        This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF. */

  uint8_t Green;                   /*!< Configures the green value.
                                        This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF. */

  uint8_t Red;                     /*!< Configures the red value.
                                        This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF. */

  uint8_t Reserved;                /*!< Reserved 0xFF */
} LTDC_ColorTypeDef;

/**
  * @brief  LTDC Init structure definition
  */
typedef struct
{
  uint32_t            HSPolarity;                /*!< configures the horizontal synchronization polarity.
                                                      This parameter can be one value of @ref LTDC_HS_POLARITY */

  uint32_t            VSPolarity;                /*!< configures the vertical synchronization polarity.
                                                      This parameter can be one value of @ref LTDC_VS_POLARITY */

  uint32_t            DEPolarity;                /*!< configures the data enable polarity.
                                                      This parameter can be one of value of @ref LTDC_DE_POLARITY */

  uint32_t            PCPolarity;                /*!< configures the pixel clock polarity.
                                                      This parameter can be one of value of @ref LTDC_PC_POLARITY */

  uint32_t            HorizontalSync;            /*!< configures the number of Horizontal synchronization width.
                                                      This parameter must be a number between Min_Data = 0x000 and Max_Data = 0xFFF. */

  uint32_t            VerticalSync;              /*!< configures the number of Vertical synchronization height.
                                                      This parameter must be a number between Min_Data = 0x000 and Max_Data = 0x7FF. */

  uint32_t            AccumulatedHBP;            /*!< configures the accumulated horizontal back porch width.
                                                      This parameter must be a number between Min_Data = LTDC_HorizontalSync and Max_Data = 0xFFF. */

  uint32_t            AccumulatedVBP;            /*!< configures the accumulated vertical back porch height.
                                                      This parameter must be a number between Min_Data = LTDC_VerticalSync and Max_Data = 0x7FF. */

  uint32_t            AccumulatedActiveW;        /*!< configures the accumulated active width.
                                                      This parameter must be a number between Min_Data = LTDC_AccumulatedHBP and Max_Data = 0xFFF. */

  uint32_t            AccumulatedActiveH;        /*!< configures the accumulated active height.
                                                      This parameter must be a number between Min_Data = LTDC_AccumulatedVBP and Max_Data = 0x7FF. */

  uint32_t            TotalWidth;                /*!< configures the total width.
                                                      This parameter must be a number between Min_Data = LTDC_AccumulatedActiveW and Max_Data = 0xFFF. */

  uint32_t            TotalHeigh;                /*!< configures the total height.
                                                      This parameter must be a number between Min_Data = LTDC_AccumulatedActiveH and Max_Data = 0x7FF. */

  LTDC_ColorTypeDef   Backcolor;                 /*!< Configures the background color. */
} LTDC_InitTypeDef;

/**
  * @brief  LTDC Layer structure definition
  */
typedef struct
{
  uint32_t WindowX0;                   /*!< Configures the Window Horizontal Start Position.
                                            This parameter must be a number between Min_Data = 0x000 and Max_Data = 0xFFF. */

  uint32_t WindowX1;                   /*!< Configures the Window Horizontal Stop Position.
                                            This parameter must be a number between Min_Data = 0x000 and Max_Data = 0xFFF. */

  uint32_t WindowY0;                   /*!< Configures the Window vertical Start Position.
                                            This parameter must be a number between Min_Data = 0x000 and Max_Data = 0x7FF. */

  uint32_t WindowY1;                   /*!< Configures the Window vertical Stop Position.
                                            This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0x7FF. */

  uint32_t PixelFormat;                /*!< Specifies the pixel format.
                                            This parameter can be one of value of @ref LTDC_Pixelformat */

  uint32_t Alpha;                      /*!< Specifies the constant alpha used for blending.
                                            This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF. */

  uint32_t Alpha0;                     /*!< Configures the default alpha value.
                                            This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF. */

  uint32_t BlendingFactor1;            /*!< Select the blending factor 1.
                                            This parameter can be one of value of @ref LTDC_BlendingFactor1 */

  uint32_t BlendingFactor2;            /*!< Select the blending factor 2.
                                            This parameter can be one of value of @ref LTDC_BlendingFactor2 */

  uint32_t FBStartAdress;              /*!< Configures the color frame buffer address */

  uint32_t ImageWidth;                 /*!< Configures the color frame buffer line length.
                                            This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0x1FFF. */

  uint32_t ImageHeight;                /*!< Specifies the number of line in frame buffer.
                                            This parameter must be a number between Min_Data = 0x000 and Max_Data = 0x7FF. */

  LTDC_ColorTypeDef   Backcolor;       /*!< Configures the layer background color. */
} LTDC_LayerCfgTypeDef;

/**
  * @brief  HAL LTDC State enumeration definition
  */
typedef enum
{
  HAL_LTDC_STATE_RESET             = 0x00U,    /*!< LTDC not yet initialized or disabled */
  HAL_LTDC_STATE_READY             = 0x01U,    /*!< LTDC initialized and ready for use   */
  HAL_LTDC_STATE_BUSY              = 0x02U,    /*!< LTDC internal process is ongoing     */
  HAL_LTDC_STATE_TIMEOUT           = 0x03U,    /*!< LTDC Timeout state                   */
  HAL_LTDC_STATE_ERROR             = 0x04U     /*!< LTDC state error                     */
}HAL_LTDC_StateTypeDef;

/**
  * @brief  LTDC handle Structure definition
  */
typedef struct
{
  LTDC_TypeDef                *Instance;                /*!< LTDC Register base address                */

  LTDC_InitTypeDef            Init;                     /*!< LTDC parameters                           */

  LTDC_LayerCfgTypeDef        LayerCfg[MAX_LAYER];      /*!< LTDC Layers parameters                    */

  HAL_LockTypeDef             Lock;                     /*!< LTDC Lock                                 */

  __IO HAL_LTDC_StateTypeDef  State;                    /*!< LTDC state                                */

  __IO uint32_t               ErrorCode;                /*!< LTDC Error code                           */

} LTDC_HandleTypeDef;

/** @defgroup LTDC_HS_POLARITY LTDC HS POLARITY
  * @{
  */
#define LTDC_HSPOLARITY_AL                (0x00000000U)                /*!< Horizontal Synchronization is active low. */
#define LTDC_HSPOLARITY_AH                LTDC_GCR_HSPOL               /*!< Horizontal Synchronization is active high. */
/**
  * @}
  */

/** @defgroup LTDC_VS_POLARITY LTDC VS POLARITY
  * @{
  */
#define LTDC_VSPOLARITY_AL                (0x00000000U)                /*!< Vertical Synchronization is active low. */
#define LTDC_VSPOLARITY_AH                LTDC_GCR_VSPOL               /*!< Vertical Synchronization is active high. */
/**
  * @}
  */

/** @defgroup LTDC_DE_POLARITY LTDC DE POLARITY
  * @{
  */
#define LTDC_DEPOLARITY_AL                (0x00000000U)                /*!< Data Enable, is active low. */
#define LTDC_DEPOLARITY_AH                LTDC_GCR_DEPOL               /*!< Data Enable, is active high. */
/**
  * @}
  */

/** @defgroup LTDC_PC_POLARITY LTDC PC POLARITY
  * @{
  */
#define LTDC_PCPOLARITY_IPC               (0x00000000U)                /*!< input pixel clock. */
#define LTDC_PCPOLARITY_IIPC              LTDC_GCR_PCPOL               /*!< inverted input pixel clock. */

/** @defgroup LTDC_Error_Code LTDC Error Code
  * @{
  */
#define HAL_LTDC_ERROR_NONE      (0x00000000U)    /*!< LTDC No error             */
#define HAL_LTDC_ERROR_TE        (0x00000001U)    /*!< LTDC Transfer error       */
#define HAL_LTDC_ERROR_FU        (0x00000002U)    /*!< LTDC FIFO Underrun        */
#define HAL_LTDC_ERROR_TIMEOUT   (0x00000020U)    /*!< LTDC Timeout error        */

#define D1_APB1PERIPH_BASE       		  (PERIPH_BASE + 0x10000000)
#define LTDC_BASE             			  (D1_APB1PERIPH_BASE + 0x1000)
#define LTDC                			  ((LTDC_TypeDef *)LTDC_BASE)

#define RCC_PERIPHCLK_LTDC             ((uint32_t)0x20000000)

#define RCC_PLLCFGR_DIVP3EN_Pos                (22U)
#define RCC_PLLCFGR_DIVP3EN_Msk                (0x1U << RCC_PLLCFGR_DIVP3EN_Pos) /*!< 0x00400000 */
#define RCC_PLLCFGR_DIVP3EN                    RCC_PLLCFGR_DIVP3EN_Msk
#define RCC_PLLCFGR_DIVQ3EN_Pos                (23U)
#define RCC_PLLCFGR_DIVQ3EN_Msk                (0x1U << RCC_PLLCFGR_DIVQ3EN_Pos) /*!< 0x00800000 */
#define RCC_PLLCFGR_DIVQ3EN                    RCC_PLLCFGR_DIVQ3EN_Msk
#define RCC_PLLCFGR_DIVR3EN_Pos                (24U)
#define RCC_PLLCFGR_DIVR3EN_Msk                (0x1U << RCC_PLLCFGR_DIVR3EN_Pos) /*!< 0x01000000 */
#define RCC_PLLCFGR_DIVR3EN                    RCC_PLLCFGR_DIVR3EN_Msk

/** @defgroup RCC_PLL3_Clock_Output  RCC PLL3 Clock Output
  * @{
  */
#define RCC_PLL3_DIVP                RCC_PLLCFGR_DIVP3EN
#define RCC_PLL3_DIVQ                RCC_PLLCFGR_DIVQ3EN
#define RCC_PLL3_DIVR                RCC_PLLCFGR_DIVR3EN

#define __HAL_RCC_PLL3CLKOUT_ENABLE(__RCC_PLL3ClockOut__)   SET_BIT(RCC->PLLCFGR, (__RCC_PLL3ClockOut__))


#define RCC_PLLCKSELR_PLLSRC_Pos               (0U)
#define RCC_PLLCKSELR_PLLSRC_Msk               (0x3U << RCC_PLLCKSELR_PLLSRC_Pos) /*!< 0x00000003 */
#define RCC_PLLCKSELR_PLLSRC                   RCC_PLLCKSELR_PLLSRC_Msk

#define __HAL_RCC_GET_PLL_OSCSOURCE() ((uint32_t)(RCC->PLLCKSELR & RCC_PLLCKSELR_PLLSRC))

/** @defgroup RCC_PLL_Clock_Source  RCC PLL Clock Source
  * @{
  */
#define RCC_PLLSOURCE_HSI              ((uint32_t)0x00000000)
#define RCC_PLLSOURCE_CSI              ((uint32_t)0x00000001)
#define RCC_PLLSOURCE_HSE              ((uint32_t)0x00000002)
#define RCC_PLLSOURCE_NONE             ((uint32_t)0x00000003)

#define RCC_CR_PLL3ON_Pos                      (28U)
#define RCC_CR_PLL3ON_Msk                      (0x1U << RCC_CR_PLL3ON_Pos)     /*!< 0x10000000 */
#define RCC_CR_PLL3ON                          RCC_CR_PLL3ON_Msk               /*!< System PLL3 clock enable */

/** @brief  Macros to enable or disable the main PLL3.
  * @note   After enabling  PLL3, the application software should wait on
  *         PLL3RDY flag to be set indicating that PLL3 clock is stable and can
  *         be used as kernel clock source.
  * @note   PLL3 is disabled by hardware when entering STOP and STANDBY modes.
  */
#define __HAL_RCC_PLL3_ENABLE()         SET_BIT(RCC->CR, RCC_CR_PLL3ON)
#define __HAL_RCC_PLL3_DISABLE()        CLEAR_BIT(RCC->CR, RCC_CR_PLL3ON)

#define RCC_PLLCKSELR_DIVM3_Pos                (20U)
#define RCC_PLLCKSELR_DIVM3_Msk                (0x3FU << RCC_PLLCKSELR_DIVM3_Pos) /*!< 0x03F00000 */
#define RCC_PLLCKSELR_DIVM3                    RCC_PLLCKSELR_DIVM3_Msk

#define RCC_PLL3DIVR_N3_Pos                    (0U)
#define RCC_PLL3DIVR_N3_Msk                    (0x1FFU << RCC_PLL3DIVR_N3_Pos) /*!< 0x000001FF */
#define RCC_PLL3DIVR_N3                        RCC_PLL3DIVR_N3_Msk

#define RCC_PLL3DIVR_P3_Pos                    (9U)
#define RCC_PLL3DIVR_P3_Msk                    (0x7FU << RCC_PLL3DIVR_P3_Pos)  /*!< 0x0000FE00 */
#define RCC_PLL3DIVR_P3                        RCC_PLL3DIVR_P3_Msk

#define RCC_PLL3DIVR_Q3_Pos                    (16U)
#define RCC_PLL3DIVR_Q3_Msk                    (0x7FU << RCC_PLL3DIVR_Q3_Pos)  /*!< 0x007F0000 */
#define RCC_PLL3DIVR_Q3                        RCC_PLL3DIVR_Q3_Msk

#define RCC_PLL3DIVR_R3_Pos                    (24U)
#define RCC_PLL3DIVR_R3_Msk                    (0x7FU << RCC_PLL3DIVR_R3_Pos)  /*!< 0x7F000000 */
#define RCC_PLL3DIVR_R3                        RCC_PLL3DIVR_R3_Msk

/**
  * @brief  Macro to configures the PLL3  multiplication and division factors.
  * @note   This function must be used only when PLL3 is disabled.
  *
  * @param  __PLL3M__: specifies the division factor for PLL3 VCO input clock
  *          This parameter must be a number between 1 and 63.
  * @note   You have to set the PLLM parameter correctly to ensure that the VCO input
  *         frequency ranges from 1 to 16 MHz.
  *
  * @param  __PLL3N__: specifies the multiplication factor for PLL3 VCO output clock
  *          This parameter must be a number between 4 and 512.
  * @note   You have to set the PLL3N parameter correctly to ensure that the VCO
  *         output frequency is between 150 and 420 MHz (when in medium VCO range) or
  *         between 192 and 836 MHZ (when in wide VCO range)
  *
  * @param  __PLL3P__: specifies the division factor for peripheral kernel clocks
  *          This parameter must be a number between 2 and 128 (where odd numbers not allowed)
  *
  * @param  __PLL3Q__: specifies the division factor for peripheral kernel clocks
  *          This parameter must be a number between 1 and 128
  *
  * @param  __PLL3R__: specifies the division factor for peripheral kernel clocks
  *          This parameter must be a number between 1 and 128
  *
  * @retval None
  */

#define __HAL_RCC_PLL3_CONFIG(__PLL3M__, __PLL3N__, __PLL3P__, __PLL3Q__,__PLL3R__ ) \
                  do{ MODIFY_REG(RCC->PLLCKSELR, ( RCC_PLLCKSELR_DIVM3) , ( (__PLL3M__) <<20U));  \
                         WRITE_REG (RCC->PLL3DIVR , ( (((__PLL3N__) - 1U ) & RCC_PLL3DIVR_N3) | ((((__PLL3P__) -1U ) << 9U) & RCC_PLL3DIVR_P3) | \
                                   ((((__PLL3Q__) -1U) << 16U) & RCC_PLL3DIVR_Q3) | ((((__PLL3R__) - 1U) << 24U) & RCC_PLL3DIVR_R3))); \
                       } while(0)


#define RCC_PLLCFGR_PLL3RGE_Pos                (10U)
#define RCC_PLLCFGR_PLL3RGE_Msk                (0x3U << RCC_PLLCFGR_PLL3RGE_Pos) /*!< 0x00000C00 */
#define RCC_PLLCFGR_PLL3RGE                    RCC_PLLCFGR_PLL3RGE_Msk

/** @brief  Macro to select  the PLL3  reference frequency range.
  * @param  __RCC_PLL3VCIRange__: specifies the PLL1 input frequency range
  *         This parameter can be one of the following values:
  *            @arg RCC_PLL3VCIRANGE_0: Range frequency is between 1 and 2 MHz
  *            @arg RCC_PLL3VCIRANGE_1: Range frequency is between 2 and 4 MHz
  *            @arg RCC_PLL3VCIRANGE_2: Range frequency is between 4 and 8 MHz
  *            @arg RCC_PLL3VCIRANGE_3: Range frequency is between 8 and 16 MHz
  * @retval None
  */
#define __HAL_RCC_PLL3_VCIRANGE(__RCC_PLL3VCIRange__) \
                  MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLL3RGE, (__RCC_PLL3VCIRange__))

#define RCC_PLLCFGR_PLL3VCOSEL_Pos             (9U)
#define RCC_PLLCFGR_PLL3VCOSEL_Msk             (0x1U << RCC_PLLCFGR_PLL3VCOSEL_Pos) /*!< 0x00000200 */
#define RCC_PLLCFGR_PLL3VCOSEL                 RCC_PLLCFGR_PLL3VCOSEL_Msk

/** @brief  Macro to select  the PLL3  reference frequency range.
  * @param  __RCC_PLL3VCORange__: specifies the PLL1 input frequency range
  *         This parameter can be one of the following values:
  *            @arg RCC_PLL3VCOWIDE: Range frequency is between 192 and 836 MHz
  *            @arg RCC_PLL3VCOMEDIUM: Range frequency is between 150 and 420 MHz
  * @retval None
  */
#define __HAL_RCC_PLL3_VCORANGE(__RCC_PLL3VCORange__) \
                  MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLL3VCOSEL, (__RCC_PLL3VCORange__))

#define LTDC_GCR_PCPOL_Pos           (28U)
#define LTDC_GCR_PCPOL_Msk           (0x1U << LTDC_GCR_PCPOL_Pos)              /*!< 0x10000000 */
#define LTDC_GCR_PCPOL               LTDC_GCR_PCPOL_Msk

#define LTDC_GCR_DEPOL_Pos           (29U)
#define LTDC_GCR_DEPOL_Msk           (0x1U << LTDC_GCR_DEPOL_Pos)              /*!< 0x20000000 */
#define LTDC_GCR_DEPOL               LTDC_GCR_DEPOL_Msk

#define LTDC_GCR_VSPOL_Pos           (30U)
#define LTDC_GCR_VSPOL_Msk           (0x1U << LTDC_GCR_VSPOL_Pos)              /*!< 0x40000000 */
#define LTDC_GCR_VSPOL               LTDC_GCR_VSPOL_Msk                        /*!< Vertical Synchronization Polarity   */

#define LTDC_GCR_HSPOL_Pos           (31U)
#define LTDC_GCR_HSPOL_Msk           (0x1U << LTDC_GCR_HSPOL_Pos)              /*!< 0x80000000 */
#define LTDC_GCR_HSPOL               LTDC_GCR_HSPOL_Msk                        /*!< Horizontal Synchronization Polarity */

#define LTDC_SSCR_VSH_Pos            (0U)
#define LTDC_SSCR_VSH_Msk            (0x7FFU << LTDC_SSCR_VSH_Pos)             /*!< 0x000007FF */
#define LTDC_SSCR_VSH                LTDC_SSCR_VSH_Msk                         /*!< Vertical Synchronization Height  */
#define LTDC_SSCR_HSW_Pos            (16U)
#define LTDC_SSCR_HSW_Msk            (0xFFFU << LTDC_SSCR_HSW_Pos)             /*!< 0x0FFF0000 */
#define LTDC_SSCR_HSW                LTDC_SSCR_HSW_Msk

#define LTDC_BPCR_AVBP_Pos           (0U)
#define LTDC_BPCR_AVBP_Msk           (0x7FFU << LTDC_BPCR_AVBP_Pos)            /*!< 0x000007FF */
#define LTDC_BPCR_AVBP               LTDC_BPCR_AVBP_Msk                        /*!< Accumulated Vertical Back Porch   */
#define LTDC_BPCR_AHBP_Pos           (16U)
#define LTDC_BPCR_AHBP_Msk           (0xFFFU << LTDC_BPCR_AHBP_Pos)            /*!< 0x0FFF0000 */
#define LTDC_BPCR_AHBP               LTDC_BPCR_AHBP_Msk

#define LTDC_AWCR_AAH_Pos            (0U)
#define LTDC_AWCR_AAH_Msk            (0x7FFU << LTDC_AWCR_AAH_Pos)             /*!< 0x000007FF */
#define LTDC_AWCR_AAH                LTDC_AWCR_AAH_Msk                         /*!< Accumulated Active heigh */
#define LTDC_AWCR_AAW_Pos            (16U)
#define LTDC_AWCR_AAW_Msk            (0xFFFU << LTDC_AWCR_AAW_Pos)             /*!< 0x0FFF0000 */
#define LTDC_AWCR_AAW                LTDC_AWCR_AAW_Msk                         /*!< Accumulated Active Width */

#define LTDC_TWCR_TOTALH_Pos         (0U)
#define LTDC_TWCR_TOTALH_Msk         (0x7FFU << LTDC_TWCR_TOTALH_Pos)          /*!< 0x000007FF */
#define LTDC_TWCR_TOTALH             LTDC_TWCR_TOTALH_Msk                      /*!< Total Heigh */
#define LTDC_TWCR_TOTALW_Pos         (16U)
#define LTDC_TWCR_TOTALW_Msk         (0xFFFU << LTDC_TWCR_TOTALW_Pos)          /*!< 0x0FFF0000 */
#define LTDC_TWCR_TOTALW             LTDC_TWCR_TOTALW_Msk                      /*!< Total Width */

#define LTDC_BCCR_BCBLUE_Pos         (0U)
#define LTDC_BCCR_BCBLUE_Msk         (0xFFU << LTDC_BCCR_BCBLUE_Pos)           /*!< 0x000000FF */
#define LTDC_BCCR_BCBLUE             LTDC_BCCR_BCBLUE_Msk                      /*!< Background Blue value  */
#define LTDC_BCCR_BCGREEN_Pos        (8U)
#define LTDC_BCCR_BCGREEN_Msk        (0xFFU << LTDC_BCCR_BCGREEN_Pos)          /*!< 0x0000FF00 */
#define LTDC_BCCR_BCGREEN            LTDC_BCCR_BCGREEN_Msk                     /*!< Background Green value */
#define LTDC_BCCR_BCRED_Pos          (16U)
#define LTDC_BCCR_BCRED_Msk          (0xFFU << LTDC_BCCR_BCRED_Pos)            /*!< 0x00FF0000 */
#define LTDC_BCCR_BCRED              LTDC_BCCR_BCRED_Msk                       /*!< Background Red value   */

/**
  * @brief  Enables the specified LTDC interrupts.
  * @param  __HANDLE__: LTDC handle
  * @param __INTERRUPT__: specifies the LTDC interrupt sources to be enabled.
  *          This parameter can be any combination of the following values:
  *            @arg LTDC_IT_LI: Line Interrupt flag
  *            @arg LTDC_IT_FU: FIFO Underrun Interrupt flag
  *            @arg LTDC_IT_TE: Transfer Error interrupt flag
  *            @arg LTDC_IT_RR: Register Reload Interrupt Flag
  * @retval None
  */
#define __HAL_LTDC_ENABLE_IT(__HANDLE__, __INTERRUPT__) ((__HANDLE__)->Instance->IER |= (__INTERRUPT__))


#define LTDC_IER_LIE_Pos             (0U)
#define LTDC_IER_LIE_Msk             (0x1U << LTDC_IER_LIE_Pos)                /*!< 0x00000001 */
#define LTDC_IER_LIE                 LTDC_IER_LIE_Msk                          /*!< Line Interrupt Enable            */
#define LTDC_IER_FUIE_Pos            (1U)
#define LTDC_IER_FUIE_Msk            (0x1U << LTDC_IER_FUIE_Pos)               /*!< 0x00000002 */
#define LTDC_IER_FUIE                LTDC_IER_FUIE_Msk                         /*!< FIFO Underrun Interrupt Enable   */
#define LTDC_IER_TERRIE_Pos          (2U)
#define LTDC_IER_TERRIE_Msk          (0x1U << LTDC_IER_TERRIE_Pos)             /*!< 0x00000004 */
#define LTDC_IER_TERRIE              LTDC_IER_TERRIE_Msk                       /*!< Transfer Error Interrupt Enable  */
#define LTDC_IER_RRIE_Pos            (3U)
#define LTDC_IER_RRIE_Msk            (0x1U << LTDC_IER_RRIE_Pos)               /*!< 0x00000008 */
#define LTDC_IER_RRIE                LTDC_IER_RRIE_Msk                         /*!< Register Reload interrupt enable */

/** @defgroup LTDC_Interrupts LTDC Interrupts
  * @{
  */
#define LTDC_IT_LI                      LTDC_IER_LIE
#define LTDC_IT_FU                      LTDC_IER_FUIE
#define LTDC_IT_TE                      LTDC_IER_TERRIE
#define LTDC_IT_RR                      LTDC_IER_RRIE

#define LTDC_GCR_LTDCEN_Pos          (0U)
#define LTDC_GCR_LTDCEN_Msk          (0x1U << LTDC_GCR_LTDCEN_Pos)             /*!< 0x00000001 */
#define LTDC_GCR_LTDCEN              LTDC_GCR_LTDCEN_Msk                       /*!< LCD-TFT controller enable bit       */

/**
  * @brief  Enable the LTDC.
  * @param  __HANDLE__: LTDC handle
  * @retval None.
  */
#define __HAL_LTDC_ENABLE(__HANDLE__)    ((__HANDLE__)->Instance->GCR |= LTDC_GCR_LTDCEN)
#define __HAL_LTDC_DISABLE(__HANDLE__)   ((__HANDLE__)->Instance->GCR &= ~(LTDC_GCR_LTDCEN))
#define __HAL_LTDC_LAYER_ENABLE(__HANDLE__, __LAYER__)  ((LTDC_LAYER((__HANDLE__), (__LAYER__)))->CR |= (uint32_t)LTDC_LxCR_LEN)
#define __HAL_LTDC_LAYER_DISABLE(__HANDLE__, __LAYER__) ((LTDC_LAYER((__HANDLE__), (__LAYER__)))->CR &= ~(uint32_t)LTDC_LxCR_LEN)
#define __HAL_LTDC_RELOAD_IMMEDIATE_CONFIG(__HANDLE__)  ((__HANDLE__)->Instance->SRCR |= LTDC_SRCR_IMR)

#define __HAL_LTDC_RELOAD_CONFIG  __HAL_LTDC_RELOAD_IMMEDIATE_CONFIG

#define GPIO_AF9_LTDC           ((uint8_t)0x09)
#define GPIO_AF12_LTDC          ((uint8_t)0xC)
#define GPIO_AF14_LTDC         ((uint8_t)0x0E)

typedef struct
{
  __IO uint32_t CR;            /*!< LTDC Layerx Control Register                                  Address offset: 0x84 */
  __IO uint32_t WHPCR;         /*!< LTDC Layerx Window Horizontal Position Configuration Register Address offset: 0x88 */
  __IO uint32_t WVPCR;         /*!< LTDC Layerx Window Vertical Position Configuration Register   Address offset: 0x8C */
  __IO uint32_t CKCR;          /*!< LTDC Layerx Color Keying Configuration Register               Address offset: 0x90 */
  __IO uint32_t PFCR;          /*!< LTDC Layerx Pixel Format Configuration Register               Address offset: 0x94 */
  __IO uint32_t CACR;          /*!< LTDC Layerx Constant Alpha Configuration Register             Address offset: 0x98 */
  __IO uint32_t DCCR;          /*!< LTDC Layerx Default Color Configuration Register              Address offset: 0x9C */
  __IO uint32_t BFCR;          /*!< LTDC Layerx Blending Factors Configuration Register           Address offset: 0xA0 */
  uint32_t      RESERVED0[2];  /*!< Reserved */
  __IO uint32_t CFBAR;         /*!< LTDC Layerx Color Frame Buffer Address Register               Address offset: 0xAC */
  __IO uint32_t CFBLR;         /*!< LTDC Layerx Color Frame Buffer Length Register                Address offset: 0xB0 */
  __IO uint32_t CFBLNR;        /*!< LTDC Layerx ColorFrame Buffer Line Number Register            Address offset: 0xB4 */
  uint32_t      RESERVED1[3];  /*!< Reserved */
  __IO uint32_t CLUTWR;         /*!< LTDC Layerx CLUT Write Register                               Address offset: 0x144 */

} LTDC_Layer_TypeDef;

#define LTDC_LAYER(__HANDLE__, __LAYER__)              ((LTDC_Layer_TypeDef *)((uint32_t)(((uint32_t)((__HANDLE__)->Instance)) + 0x84 + (0x80*(__LAYER__)))))

#define LTDC_LxWHPCR_WHSTPOS_Pos     (0U)
#define LTDC_LxWHPCR_WHSTPOS_Msk     (0xFFFU << LTDC_LxWHPCR_WHSTPOS_Pos)      /*!< 0x00000FFF */
#define LTDC_LxWHPCR_WHSTPOS         LTDC_LxWHPCR_WHSTPOS_Msk                  /*!< Window Horizontal Start Position */

#define LTDC_LxWHPCR_WHSPPOS_Pos     (16U)
#define LTDC_LxWHPCR_WHSPPOS_Msk     (0xFFFFU << LTDC_LxWHPCR_WHSPPOS_Pos)     /*!< 0xFFFF0000 */
#define LTDC_LxWHPCR_WHSPPOS         LTDC_LxWHPCR_WHSPPOS_Msk                  /*!< Window Horizontal Stop Position  */

#define LTDC_LxWVPCR_WVSTPOS_Pos     (0U)
#define LTDC_LxWVPCR_WVSTPOS_Msk     (0xFFFU << LTDC_LxWVPCR_WVSTPOS_Pos)      /*!< 0x00000FFF */
#define LTDC_LxWVPCR_WVSTPOS         LTDC_LxWVPCR_WVSTPOS_Msk                  /*!< Window Vertical Start Position */
#define LTDC_LxWVPCR_WVSPPOS_Pos     (16U)
#define LTDC_LxWVPCR_WVSPPOS_Msk     (0xFFFFU << LTDC_LxWVPCR_WVSPPOS_Pos)     /*!< 0xFFFF0000 */
#define LTDC_LxWVPCR_WVSPPOS         LTDC_LxWVPCR_WVSPPOS_Msk                  /*!< Window Vertical Stop Position  */

#define LTDC_LxPFCR_PF_Pos           (0U)
#define LTDC_LxPFCR_PF_Msk           (0x7U << LTDC_LxPFCR_PF_Pos)              /*!< 0x00000007 */
#define LTDC_LxPFCR_PF               LTDC_LxPFCR_PF_Msk                        /*!< Pixel Format */

#define LTDC_LxDCCR_DCBLUE_Pos       (0U)
#define LTDC_LxDCCR_DCBLUE_Msk       (0xFFU << LTDC_LxDCCR_DCBLUE_Pos)         /*!< 0x000000FF */
#define LTDC_LxDCCR_DCBLUE           LTDC_LxDCCR_DCBLUE_Msk                    /*!< Default Color Blue  */
#define LTDC_LxDCCR_DCGREEN_Pos      (8U)
#define LTDC_LxDCCR_DCGREEN_Msk      (0xFFU << LTDC_LxDCCR_DCGREEN_Pos)        /*!< 0x0000FF00 */
#define LTDC_LxDCCR_DCGREEN          LTDC_LxDCCR_DCGREEN_Msk                   /*!< Default Color Green */
#define LTDC_LxDCCR_DCRED_Pos        (16U)
#define LTDC_LxDCCR_DCRED_Msk        (0xFFU << LTDC_LxDCCR_DCRED_Pos)          /*!< 0x00FF0000 */
#define LTDC_LxDCCR_DCRED            LTDC_LxDCCR_DCRED_Msk                     /*!< Default Color Red   */
#define LTDC_LxDCCR_DCALPHA_Pos      (24U)
#define LTDC_LxDCCR_DCALPHA_Msk      (0xFFU << LTDC_LxDCCR_DCALPHA_Pos)        /*!< 0xFF000000 */
#define LTDC_LxDCCR_DCALPHA          LTDC_LxDCCR_DCALPHA_Msk                   /*!< Default Color Alpha */

#define LTDC_LxCACR_CONSTA_Pos       (0U)
#define LTDC_LxCACR_CONSTA_Msk       (0xFFU << LTDC_LxCACR_CONSTA_Pos)         /*!< 0x000000FF */
#define LTDC_LxCACR_CONSTA           LTDC_LxCACR_CONSTA_Msk                    /*!< Constant Alpha */

#define LTDC_LxBFCR_BF2_Pos          (0U)
#define LTDC_LxBFCR_BF2_Msk          (0x7U << LTDC_LxBFCR_BF2_Pos)             /*!< 0x00000007 */
#define LTDC_LxBFCR_BF2              LTDC_LxBFCR_BF2_Msk                       /*!< Blending Factor 2 */
#define LTDC_LxBFCR_BF1_Pos          (8U)
#define LTDC_LxBFCR_BF1_Msk          (0x7U << LTDC_LxBFCR_BF1_Pos)             /*!< 0x00000700 */
#define LTDC_LxBFCR_BF1              LTDC_LxBFCR_BF1_Msk                       /*!< Blending Factor 1 */

#define LTDC_LxCFBAR_CFBADD_Pos      (0U)
#define LTDC_LxCFBAR_CFBADD_Msk      (0xFFFFFFFFU << LTDC_LxCFBAR_CFBADD_Pos)  /*!< 0xFFFFFFFF */
#define LTDC_LxCFBAR_CFBADD          LTDC_LxCFBAR_CFBADD_Msk                   /*!< Color Frame Buffer Start Address */

#define LTDC_PIXEL_FORMAT_ARGB8888                  (0x00000000U)      /*!< ARGB8888 LTDC pixel format */
#define LTDC_PIXEL_FORMAT_RGB888                    (0x00000001U)      /*!< RGB888 LTDC pixel format   */
#define LTDC_PIXEL_FORMAT_RGB565                    (0x00000002U)      /*!< RGB565 LTDC pixel format   */
#define LTDC_PIXEL_FORMAT_ARGB1555                  (0x00000003U)      /*!< ARGB1555 LTDC pixel format */
#define LTDC_PIXEL_FORMAT_ARGB4444                  (0x00000004U)      /*!< ARGB4444 LTDC pixel format */
#define LTDC_PIXEL_FORMAT_L8                        (0x00000005U)      /*!< L8 LTDC pixel format       */
#define LTDC_PIXEL_FORMAT_AL44                      (0x00000006U)      /*!< AL44 LTDC pixel format     */
#define LTDC_PIXEL_FORMAT_AL88                      (0x00000007U)      /*!< AL88 LTDC pixel format     */

#define LTDC_LxCFBLR_CFBLL_Pos       (0U)
#define LTDC_LxCFBLR_CFBLL_Msk       (0x1FFFU << LTDC_LxCFBLR_CFBLL_Pos)       /*!< 0x00001FFF */
#define LTDC_LxCFBLR_CFBLL           LTDC_LxCFBLR_CFBLL_Msk                    /*!< Color Frame Buffer Line Length    */
#define LTDC_LxCFBLR_CFBP_Pos        (16U)
#define LTDC_LxCFBLR_CFBP_Msk        (0x1FFFU << LTDC_LxCFBLR_CFBP_Pos)        /*!< 0x1FFF0000 */
#define LTDC_LxCFBLR_CFBP            LTDC_LxCFBLR_CFBP_Msk                     /*!< Color Frame Buffer Pitch in bytes */

#define LTDC_LxCFBLNR_CFBLNBR_Pos    (0U)
#define LTDC_LxCFBLNR_CFBLNBR_Msk    (0x7FFU << LTDC_LxCFBLNR_CFBLNBR_Pos)     /*!< 0x000007FF */
#define LTDC_LxCFBLNR_CFBLNBR        LTDC_LxCFBLNR_CFBLNBR_Msk                 /*!< Frame Buffer Line Number */

#define LTDC_LxCR_LEN_Pos            (0U)
#define LTDC_LxCR_LEN_Msk            (0x1U << LTDC_LxCR_LEN_Pos)               /*!< 0x00000001 */
#define LTDC_LxCR_LEN                LTDC_LxCR_LEN_Msk                         /*!< Layer Enable              */

#define LTDC_SRCR_IMR_Pos            (0U)
#define LTDC_SRCR_IMR_Msk            (0x1U << LTDC_SRCR_IMR_Pos)               /*!< 0x00000001 */
#define LTDC_SRCR_IMR                LTDC_SRCR_IMR_Msk                         /*!< Immediate Reload         */

#define LTDC_LxCR_COLKEN_Pos         (1U)
#define LTDC_LxCR_COLKEN_Msk         (0x1U << LTDC_LxCR_COLKEN_Pos)            /*!< 0x00000002 */
#define LTDC_LxCR_COLKEN             LTDC_LxCR_COLKEN_Msk                      /*!< Color Keying Enable       */

#define LTDC_LxCKCR_CKBLUE_Pos       (0U)
#define LTDC_LxCKCR_CKBLUE_Msk       (0xFFU << LTDC_LxCKCR_CKBLUE_Pos)         /*!< 0x000000FF */
#define LTDC_LxCKCR_CKBLUE           LTDC_LxCKCR_CKBLUE_Msk                    /*!< Color Key Blue value  */
#define LTDC_LxCKCR_CKGREEN_Pos      (8U)
#define LTDC_LxCKCR_CKGREEN_Msk      (0xFFU << LTDC_LxCKCR_CKGREEN_Pos)        /*!< 0x0000FF00 */
#define LTDC_LxCKCR_CKGREEN          LTDC_LxCKCR_CKGREEN_Msk                   /*!< Color Key Green value */
#define LTDC_LxCKCR_CKRED_Pos        (16U)
#define LTDC_LxCKCR_CKRED_Msk        (0xFFU << LTDC_LxCKCR_CKRED_Pos)          /*!< 0x00FF0000 */
#define LTDC_LxCKCR_CKRED            LTDC_LxCKCR_CKRED_Msk                     /*!< Color Key Red value   */

#define __HAL_LTDC_LAYER LTDC_LAYER

#define __HAL_LTDC_ENABLE_IT(__HANDLE__, __INTERRUPT__) ((__HANDLE__)->Instance->IER |= (__INTERRUPT__))
#define __HAL_LTDC_DISABLE_IT(__HANDLE__, __INTERRUPT__) ((__HANDLE__)->Instance->IER &= ~(__INTERRUPT__))

#define LTDC_BLENDING_FACTOR1_CA                       (0x00000400U)   /*!< Blending factor : Cte Alpha */
#define LTDC_BLENDING_FACTOR1_PAxCA                    (0x00000600U)   /*!< Blending factor : Cte Alpha x Pixel Alpha*/

#define LTDC_BLENDING_FACTOR2_CA                       (0x00000005U)   /*!< Blending factor : Cte Alpha */
#define LTDC_BLENDING_FACTOR2_PAxCA                    (0x00000007U)   /*!< Blending factor : Cte Alpha x Pixel Alpha*/

#define LTDC_LxCR_CLUTEN_Pos         (4U)
#define LTDC_LxCR_CLUTEN_Msk         (0x1U << LTDC_LxCR_CLUTEN_Pos)            /*!< 0x00000010 */
#define LTDC_LxCR_CLUTEN             LTDC_LxCR_CLUTEN_Msk                      /*!< Color Lockup Table Enable */

typedef struct
{
  __IO uint32_t CR;            /*!< DMA2D Control Register,                         Address offset: 0x00 */
  __IO uint32_t ISR;           /*!< DMA2D Interrupt Status Register,                Address offset: 0x04 */
  __IO uint32_t IFCR;          /*!< DMA2D Interrupt Flag Clear Register,            Address offset: 0x08 */
  __IO uint32_t FGMAR;         /*!< DMA2D Foreground Memory Address Register,       Address offset: 0x0C */
  __IO uint32_t FGOR;          /*!< DMA2D Foreground Offset Register,               Address offset: 0x10 */
  __IO uint32_t BGMAR;         /*!< DMA2D Background Memory Address Register,       Address offset: 0x14 */
  __IO uint32_t BGOR;          /*!< DMA2D Background Offset Register,               Address offset: 0x18 */
  __IO uint32_t FGPFCCR;       /*!< DMA2D Foreground PFC Control Register,          Address offset: 0x1C */
  __IO uint32_t FGCOLR;        /*!< DMA2D Foreground Color Register,                Address offset: 0x20 */
  __IO uint32_t BGPFCCR;       /*!< DMA2D Background PFC Control Register,          Address offset: 0x24 */
  __IO uint32_t BGCOLR;        /*!< DMA2D Background Color Register,                Address offset: 0x28 */
  __IO uint32_t FGCMAR;        /*!< DMA2D Foreground CLUT Memory Address Register,  Address offset: 0x2C */
  __IO uint32_t BGCMAR;        /*!< DMA2D Background CLUT Memory Address Register,  Address offset: 0x30 */
  __IO uint32_t OPFCCR;        /*!< DMA2D Output PFC Control Register,              Address offset: 0x34 */
  __IO uint32_t OCOLR;         /*!< DMA2D Output Color Register,                    Address offset: 0x38 */
  __IO uint32_t OMAR;          /*!< DMA2D Output Memory Address Register,           Address offset: 0x3C */
  __IO uint32_t OOR;           /*!< DMA2D Output Offset Register,                   Address offset: 0x40 */
  __IO uint32_t NLR;           /*!< DMA2D Number of Line Register,                  Address offset: 0x44 */
  __IO uint32_t LWR;           /*!< DMA2D Line Watermark Register,                  Address offset: 0x48 */
  __IO uint32_t AMTCR;         /*!< DMA2D AHB Master Timer Configuration Register,  Address offset: 0x4C */
  uint32_t      RESERVED[236]; /*!< Reserved, 0x50-0x3FF */
  __IO uint32_t FGCLUT[256];   /*!< DMA2D Foreground CLUT,                          Address offset:400-7FF */
  __IO uint32_t BGCLUT[256];   /*!< DMA2D Background CLUT,                          Address offset:800-BFF */
} DMA2D_TypeDef;

typedef struct
{
  uint32_t             Mode;               /*!< Configures the DMA2D transfer mode.
                                                This parameter can be one value of @ref DMA2D_Mode. */

  uint32_t             ColorMode;          /*!< Configures the color format of the output image.
                                                This parameter can be one value of @ref DMA2D_Output_Color_Mode. */

  uint32_t             OutputOffset;       /*!< Specifies the Offset value.
                                                This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0x3FFF. */
  uint32_t             AlphaInverted;     /*!< Select regular or inverted alpha value for the output pixel format converter.
                                               This parameter can be one value of @ref DMA2D_Alpha_Inverted */

  uint32_t             RedBlueSwap;       /*!< Select regular mode (RGB or ARGB) or swap mode (BGR or ABGR)
                                               for the output pixel format converter.
                                               This parameter can be one value of @ref DMA2D_RB_Swap. */

} DMA2D_InitTypeDef;

typedef struct
{
  uint32_t             InputOffset;       /*!< Configures the DMA2D foreground or background offset.
                                               This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0x3FFF. */

  uint32_t             InputColorMode;    /*!< Configures the DMA2D foreground or background color mode.
                                               This parameter can be one value of @ref DMA2D_Input_Color_Mode. */

  uint32_t             AlphaMode;         /*!< Configures the DMA2D foreground or background alpha mode.
                                               This parameter can be one value of @ref DMA2D_Alpha_Mode. */

  uint32_t             InputAlpha;        /*!< Specifies the DMA2D foreground or background alpha value and color value in case of A8 or A4 color mode.
                                               This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF except for the color modes detailed below.
                                               @note In case of A8 or A4 color mode (ARGB), this parameter must be a number between
                                               Min_Data = 0x00000000 and Max_Data = 0xFFFFFFFF where
                                               - InputAlpha[24:31] is the alpha value ALPHA[0:7]
                                               - InputAlpha[16:23] is the red value RED[0:7]
                                               - InputAlpha[8:15] is the green value GREEN[0:7]
                                               - InputAlpha[0:7] is the blue value BLUE[0:7]. */

  uint32_t             AlphaInverted;     /*!< Select regular or inverted alpha value.
                                               This parameter can be one value of @ref DMA2D_Alpha_Inverted.*/

  uint32_t             RedBlueSwap;       /*!< Select regular mode (RGB or ARGB) or swap mode (BGR or ABGR).
                                               This parameter can be one value of @ref DMA2D_RB_Swap. */

  uint32_t             ChromaSubSampling; /*!< Configure the chroma sub-sampling mode for the YCbCr color mode
                                               This parameter can be one value of @ref DMA2D_Chroma_Sub_Sampling */
} DMA2D_LayerCfgTypeDef;

#define MAX_DMA2D_LAYER  2U  /*!< DMA2D maximum number of layers */

typedef enum
{
  HAL_DMA2D_STATE_RESET             = 0x00U,    /*!< DMA2D not yet initialized or disabled       */
  HAL_DMA2D_STATE_READY             = 0x01U,    /*!< Peripheral Initialized and ready for use    */
  HAL_DMA2D_STATE_BUSY              = 0x02U,    /*!< An internal process is ongoing              */
  HAL_DMA2D_STATE_TIMEOUT           = 0x03U,    /*!< Timeout state                               */
  HAL_DMA2D_STATE_ERROR             = 0x04U,    /*!< DMA2D state error                           */
  HAL_DMA2D_STATE_SUSPEND           = 0x05U     /*!< DMA2D process is suspended                  */
}HAL_DMA2D_StateTypeDef;

typedef struct __DMA2D_HandleTypeDef
{
  DMA2D_TypeDef               *Instance;                                                    /*!< DMA2D register base address.               */

  DMA2D_InitTypeDef           Init;                                                         /*!< DMA2D communication parameters.            */

  void                        (* XferCpltCallback)(struct __DMA2D_HandleTypeDef * hdma2d);  /*!< DMA2D transfer complete callback.          */

  void                        (* XferErrorCallback)(struct __DMA2D_HandleTypeDef * hdma2d); /*!< DMA2D transfer error callback.             */

  DMA2D_LayerCfgTypeDef       LayerCfg[MAX_DMA2D_LAYER];                                    /*!< DMA2D Layers parameters           */

  HAL_LockTypeDef             Lock;                                                         /*!< DMA2D lock.                                */

  __IO HAL_DMA2D_StateTypeDef State;                                                        /*!< DMA2D transfer state.                      */

  __IO uint32_t               ErrorCode;                                                    /*!< DMA2D error code.                          */
} DMA2D_HandleTypeDef;

#define DMA2D_CR_MODE_Pos          (16U)
#define DMA2D_CR_MODE_Msk          (0x3U << DMA2D_CR_MODE_Pos)                 /*!< 0x00030000 */
#define DMA2D_CR_MODE              DMA2D_CR_MODE_Msk                           /*!< DMA2D Mode[1:0]                         */
#define DMA2D_CR_MODE_0            (0x1U << DMA2D_CR_MODE_Pos)                 /*!< 0x00010000 */
#define DMA2D_CR_MODE_1            (0x2U << DMA2D_CR_MODE_Pos)                 /*!< 0x00020000 */

#define DMA2D_M2M                   ((uint32_t)0x00000000U)  /*!< DMA2D memory to memory transfer mode */
#define DMA2D_M2M_PFC               DMA2D_CR_MODE_0          /*!< DMA2D memory to memory with pixel format conversion transfer mode */
#define DMA2D_M2M_BLEND             DMA2D_CR_MODE_1          /*!< DMA2D memory to memory with blending transfer mode */
#define DMA2D_R2M                   DMA2D_CR_MODE            /*!< DMA2D register to memory transfer mode */

#define DMA2D_INPUT_ARGB8888        ((uint32_t)0x00000000U)  /*!< ARGB8888 color mode */
#define DMA2D_INPUT_RGB888          ((uint32_t)0x00000001U)  /*!< RGB888 color mode   */
#define DMA2D_INPUT_RGB565          ((uint32_t)0x00000002U)  /*!< RGB565 color mode   */
#define DMA2D_INPUT_ARGB1555        ((uint32_t)0x00000003U)  /*!< ARGB1555 color mode */
#define DMA2D_INPUT_ARGB4444        ((uint32_t)0x00000004U)  /*!< ARGB4444 color mode */
#define DMA2D_INPUT_L8              ((uint32_t)0x00000005U)  /*!< L8 color mode       */
#define DMA2D_INPUT_AL44            ((uint32_t)0x00000006U)  /*!< AL44 color mode     */
#define DMA2D_INPUT_AL88            ((uint32_t)0x00000007U)  /*!< AL88 color mode     */
#define DMA2D_INPUT_L4              ((uint32_t)0x00000008U)  /*!< L4 color mode       */
#define DMA2D_INPUT_A8              ((uint32_t)0x00000009U)  /*!< A8 color mode       */
#define DMA2D_INPUT_A4              ((uint32_t)0x0000000AU)  /*!< A4 color mode       */
#define DMA2D_INPUT_YCBCR           ((uint32_t)0x0000000BU)   /*!< YCbCr color mode */

#define D1_AHB1PERIPH_BASE      	(PERIPH_BASE + 0x12000000)
#define DMA2D_BASE            		(D1_AHB1PERIPH_BASE + 0x1000)
#define DMA2D               		((DMA2D_TypeDef *) DMA2D_BASE)

#define DMA2D_OPFCCR_CM_Pos        (0U)
#define DMA2D_OPFCCR_CM_Msk        (0x7U << DMA2D_OPFCCR_CM_Pos)               /*!< 0x00000007 */
#define DMA2D_OPFCCR_CM            DMA2D_OPFCCR_CM_Msk                         /*!< Output Color mode CM[2:0] */

#define DMA2D_OOR_LO_Pos           (0U)
#define DMA2D_OOR_LO_Msk           (0x3FFFU << DMA2D_OOR_LO_Pos)               /*!< 0x00003FFF */
#define DMA2D_OOR_LO               DMA2D_OOR_LO_Msk                            /*!< Output Line Offset */

#define DMA2D_OPFCCR_AI_Pos        (20U)
#define DMA2D_OPFCCR_AI_Msk        (0x1U << DMA2D_OPFCCR_AI_Pos)               /*!< 0x00100000 */
#define DMA2D_OPFCCR_AI            DMA2D_OPFCCR_AI_Msk                         /*!< Output Alpha Inverted */

#define DMA2D_OPFCCR_AI_Pos        (20U)
#define DMA2D_OPFCCR_AI_Msk        (0x1U << DMA2D_OPFCCR_AI_Pos)               /*!< 0x00100000 */
#define DMA2D_OPFCCR_AI            DMA2D_OPFCCR_AI_Msk                         /*!< Output Alpha Inverted */

#define DMA2D_POSITION_OPFCCR_AI      (uint32_t)POSITION_VAL(DMA2D_OPFCCR_AI)
#define DMA2D_POSITION_OPFCCR_RBS     (uint32_t)POSITION_VAL(DMA2D_OPFCCR_RBS)

#define DMA2D_OPFCCR_RBS_Pos       (21U)
#define DMA2D_OPFCCR_RBS_Msk       (0x1U << DMA2D_OPFCCR_RBS_Pos)              /*!< 0x00200000 */
#define DMA2D_OPFCCR_RBS           DMA2D_OPFCCR_RBS_Msk                        /*!< Output Red Blue Swap */

#define HAL_DMA2D_ERROR_NONE        ((uint32_t)0x00000000U)  /*!< No error             */
#define HAL_DMA2D_ERROR_TE          ((uint32_t)0x00000001U)  /*!< Transfer error       */
#define HAL_DMA2D_ERROR_CE          ((uint32_t)0x00000002U)  /*!< Configuration error  */
#define HAL_DMA2D_ERROR_CAE         ((uint32_t)0x00000004U)  /*!< CLUT access error    */
#define HAL_DMA2D_ERROR_TIMEOUT     ((uint32_t)0x00000020U)  /*!< Timeout error        */

#define DMA2D_CR_START_Pos         (0U)
#define DMA2D_CR_START_Msk         (0x1U << DMA2D_CR_START_Pos)                /*!< 0x00000001 */
#define DMA2D_CR_START             DMA2D_CR_START_Msk                          /*!< Start transfer                          */

#define __HAL_LTDC_GET_FLAG(__HANDLE__, __FLAG__) ((__HANDLE__)->Instance->ISR & (__FLAG__))
#define __HAL_LTDC_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__) ((__HANDLE__)->Instance->IER & (__INTERRUPT__))
#define __HAL_LTDC_CLEAR_FLAG(__HANDLE__, __FLAG__) ((__HANDLE__)->Instance->ICR = (__FLAG__))

#define LTDC_ISR_LIF_Pos             (0U)
#define LTDC_ISR_LIF_Msk             (0x1U << LTDC_ISR_LIF_Pos)                /*!< 0x00000001 */
#define LTDC_ISR_LIF                 LTDC_ISR_LIF_Msk                          /*!< Line Interrupt Flag */
#define LTDC_ISR_FUIF_Pos            (1U)
#define LTDC_ISR_FUIF_Msk            (0x1U << LTDC_ISR_FUIF_Pos)               /*!< 0x00000002 */
#define LTDC_ISR_FUIF                LTDC_ISR_FUIF_Msk                         /*!< FIFO Underrun Interrupt Flag */
#define LTDC_ISR_TERRIF_Pos          (2U)
#define LTDC_ISR_TERRIF_Msk          (0x1U << LTDC_ISR_TERRIF_Pos)             /*!< 0x00000004 */
#define LTDC_ISR_TERRIF              LTDC_ISR_TERRIF_Msk                       /*!< Transfer Error Interrupt Flag */
#define LTDC_ISR_RRIF_Pos            (3U)
#define LTDC_ISR_RRIF_Msk            (0x1U << LTDC_ISR_RRIF_Pos)               /*!< 0x00000008 */
#define LTDC_ISR_RRIF                LTDC_ISR_RRIF_Msk                         /*!< Register Reload interrupt Flag */

#define LTDC_FLAG_LI                     LTDC_ISR_LIF
#define LTDC_FLAG_FU                     LTDC_ISR_FUIF
#define LTDC_FLAG_TE                     LTDC_ISR_TERRIF
#define LTDC_FLAG_RR                     LTDC_ISR_RRIF

#define __NVIC_PRIO_BITS          4

// ----------------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------

void LTDC_DeInit(LTDC_HandleTypeDef *hltdc);
void LTDC_InitPLL(RCC_PeriphCLKInitTypeDef  *PeriphClkInit);
void LTDC_Init(LTDC_HandleTypeDef *hltdc);

void LTDC_SetAddress(LTDC_HandleTypeDef *hltdc, uint32_t Address, uint32_t LayerIdx);
void LTDC_ConfigCLUT(LTDC_HandleTypeDef *hltdc, uint32_t *pCLUT, uint32_t CLUTSize, uint32_t LayerIdx);
void LTDC_SetWindowPosition(LTDC_HandleTypeDef *hltdc, uint32_t X0, uint32_t Y0, uint32_t LayerIdx);
void LTDC_EnableColorKeying(LTDC_HandleTypeDef *hltdc, uint32_t LayerIdx);
void LTDC_DisableColorKeying(LTDC_HandleTypeDef *hltdc, uint32_t LayerIdx);
void LTDC_ConfigColorKeying(LTDC_HandleTypeDef *hltdc, uint32_t RGBValue, uint32_t LayerIdx);
void LTDC_ProgramLineEvent(LTDC_HandleTypeDef *hltdc, uint32_t Line);
void LTDC_ConfigLayer(LTDC_HandleTypeDef *hltdc, LTDC_LayerCfgTypeDef *pLayerCfg, uint32_t LayerIdx);
void LTDC_EnableCLUT(LTDC_HandleTypeDef *hltdc, uint32_t LayerIdx);
void DMA2D_Init(DMA2D_HandleTypeDef *hdma2d);
void LTDC_SetAlpha(LTDC_HandleTypeDef *hltdc, uint32_t Alpha, uint32_t LayerIdx);
void LTDC_IRQHandlerA(LTDC_HandleTypeDef *hltdc);

#endif
