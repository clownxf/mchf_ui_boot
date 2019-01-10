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

#ifndef __GPIO_H
#define __GPIO_H

#include "main.h"

/**
  * @brief  PLL2 Clock structure definition
  */
typedef struct
{

  uint32_t PLL2M;       /*!< PLL2M: Division factor for PLL2 VCO input clock.
                             This parameter must be a number between Min_Data = 1 and Max_Data = 63    */

  uint32_t PLL2N;       /*!< PLL2N: Multiplication factor for PLL2 VCO output clock.
                             This parameter must be a number between Min_Data = 4 and Max_Data = 512   */

  uint32_t PLL2P;       /*!< PLL2P: Division factor for system clock.
                             This parameter must be a number between Min_Data = 2 and Max_Data = 128
                             odd division factors are not allowed                                      */

  uint32_t PLL2Q;        /*!< PLL2Q: Division factor for peripheral clocks.
                             This parameter must be a number between Min_Data = 1 and Max_Data = 128   */

  uint32_t PLL2R;        /*!< PLL2R: Division factor for peripheral clocks.
                             This parameter must be a number between Min_Data = 1 and Max_Data = 128   */
  uint32_t PLL2RGE;      /*!<PLL2RGE: PLL2 clock Input range
                          This parameter must be a value of @ref RCC_PLL2_VCI_Range                    */
  uint32_t PLL2VCOSEL;   /*!<PLL2VCOSEL: PLL2 clock Output range
                          This parameter must be a value of @ref RCC_PLL2_VCO_Range                    */

  uint32_t PLL2FRACN;    /*!<PLL2FRACN: Specifies Fractional Part Of The Multiplication Factor for
                            PLL2 VCO It should be a value between 0 and 8191                           */
}RCC_PLL2InitTypeDef;


/**
  * @brief  PLL3 Clock structure definition
  */
typedef struct
{

  uint32_t PLL3M;       /*!< PLL3M: Division factor for PLL3 VCO input clock.
                             This parameter must be a number between Min_Data = 1 and Max_Data = 63    */

  uint32_t PLL3N;       /*!< PLL3N: Multiplication factor for PLL3 VCO output clock.
                             This parameter must be a number between Min_Data = 4 and Max_Data = 512   */

  uint32_t PLL3P;       /*!< PLL3P: Division factor for system clock.
                             This parameter must be a number between Min_Data = 2 and Max_Data = 128
                             odd division factors are not allowed                                      */

  uint32_t PLL3Q;        /*!< PLL3Q: Division factor for peripheral clocks.
                             This parameter must be a number between Min_Data = 1 and Max_Data = 128   */

  uint32_t PLL3R;        /*!< PLL3R: Division factor for peripheral clocks.
                             This parameter must be a number between Min_Data = 1 and Max_Data = 128   */
  uint32_t PLL3RGE;      /*!<PLL3RGE: PLL3 clock Input range
                          This parameter must be a value of @ref RCC_PLL3_VCI_Range                    */
  uint32_t PLL3VCOSEL;   /*!<PLL3VCOSEL: PLL3 clock Output range
                          This parameter must be a value of @ref RCC_PLL3_VCO_Range                    */

  uint32_t PLL3FRACN;    /*!<PLL3FRACN: Specifies Fractional Part Of The Multiplication Factor for
                            PLL3 VCO It should be a value between 0 and 8191                           */
}RCC_PLL3InitTypeDef;

/**
  * @brief  RCC extended clocks structure definition
  */
typedef struct
{
  uint32_t PeriphClockSelection;   /*!< The Extended Clock to be configured.
                                        This parameter can be a value of @ref RCCEx_Periph_Clock_Selection */

  RCC_PLL2InitTypeDef PLL2;        /*!< PLL2structure parameters.
                                        This parameter will be used only when PLL2 is selected as kernel clock Source for some peripherals */

  RCC_PLL3InitTypeDef PLL3;        /*!< PLL3 structure parameters.
                                        This parameter will be used only when PLL2 is selected as kernel clock Source for some peripherals */

  uint32_t FmcClockSelection;     /*!< Specifies FMC clock source
                                        This parameter can be a value of @ref RCCEx_FMC_Clock_Source     */

  uint32_t QspiClockSelection;    /*!< Specifies QSPI clock source
                                        This parameter can be a value of @ref RCCEx_QSPI_Clock_Source    */

  uint32_t SdmmcClockSelection;    /*!< Specifies SDMMC clock source
                                        This parameter can be a value of @ref RCCEx_SDMMC_Clock_Source   */

  uint32_t CkperClockSelection;   /*!< Specifies CKPER clock source
                                        This parameter can be a value of @ref RCCEx_CLKP_Clock_Source   */

  uint32_t Sai1ClockSelection;     /*!< Specifies SAI1 clock source
                                        This parameter can be a value of @ref RCCEx_SAI1_Clock_Source    */

  uint32_t Sai23ClockSelection;     /*!< Specifies SAI2/3 clock source
                                         This parameter can be a value of @ref RCCEx_SAI23_Clock_Source    */

  uint32_t Spi123ClockSelection;     /*!< Specifies SPI1/2/3 clock source
                                          This parameter can be a value of @ref RCCEx_SPI123_Clock_Source    */

  uint32_t Spi45ClockSelection;     /*!< Specifies SPI4/5 clock source
                                         This parameter can be a value of @ref RCCEx_SPI45_Clock_Source    */

  uint32_t SpdifrxClockSelection;   /*!< Specifies SPDIFRX Clock clock source
                                        This parameter can be a value of @ref RCCEx_SPDIFRX_Clock_Source */

  uint32_t Dfsdm1ClockSelection;    /*!< Specifies DFSDM1 Clock clock source
                                        This parameter can be a value of @ref RCCEx_DFSDM1_Clock_Source  */
#if defined(FDCAN1) || defined(FDCAN2)
  uint32_t FdcanClockSelection;   /*!< Specifies FDCAN Clock clock source
                                        This parameter can be a value of @ref RCCEx_FDCAN_Clock_Source   */
#endif /*FDCAN1 || FDCAN2*/

  uint32_t Swpmi1ClockSelection;   /*!< Specifies SWPMI1 Clock clock source
                                        This parameter can be a value of @ref RCCEx_SWPMI1_Clock_Source  */

  uint32_t Usart234578ClockSelection;   /*!< Specifies USART2/3/4/5/7/8 clock source
                                             This parameter can be a value of @ref RCCEx_USART234578_Clock_Source  */

  uint32_t Usart16ClockSelection;  /*!< Specifies USART1/6 clock source
                                        This parameter can be a value of @ref RCCEx_USART16_Clock_Source  */

  uint32_t RngClockSelection;      /*!< Specifies RNG clock source
                                        This parameter can be a value of @ref RCCEx_RNG_Clock_Source     */

  uint32_t I2c123ClockSelection;   /*!< Specifies I2C1/2/3 clock source
                                        This parameter can be a value of @ref RCCEx_I2C123_Clock_Source    */

  uint32_t UsbClockSelection;      /*!< Specifies USB clock source
                                        This parameter can be a value of @ref RCCEx_USB_Clock_Source     */

  uint32_t CecClockSelection;     /*!< Specifies CEC clock source
                                        This parameter can be a value of @ref RCCEx_CEC_Clock_Source     */

  uint32_t Lptim1ClockSelection;   /*!< Specifies LPTIM1 clock source
                                        This parameter can be a value of @ref RCCEx_LPTIM1_Clock_Source  */

  uint32_t Lpuart1ClockSelection;  /*!< Specifies LPUART1 clock source
                                        This parameter can be a value of @ref RCCEx_LPUART1_Clock_Source */

  uint32_t I2c4ClockSelection;     /*!< Specifies I2C4 clock source
                                        This parameter can be a value of @ref RCCEx_I2C4_Clock_Source    */

  uint32_t Lptim2ClockSelection;   /*!< Specifies LPTIM2 clock source
                                        This parameter can be a value of @ref RCCEx_LPTIM2_Clock_Source  */

  uint32_t Lptim345ClockSelection;   /*!< Specifies LPTIM3/4/5 clock source
                                          This parameter can be a value of @ref RCCEx_LPTIM345_Clock_Source  */

  uint32_t AdcClockSelection;      /*!< Specifies ADC interface clock source
                                        This parameter can be a value of @ref RCCEx_ADC_Clock_Source     */

  uint32_t Sai4AClockSelection;     /*!< Specifies SAI4A clock source
                                        This parameter can be a value of @ref RCCEx_SAI4A_Clock_Source   */

  uint32_t Sai4BClockSelection;     /*!< Specifies SAI4B clock source
                                        This parameter can be a value of @ref RCCEx_SAI4B_Clock_Source   */

  uint32_t Spi6ClockSelection;     /*!< Specifies SPI6 clock source
                                        This parameter can be a value of @ref RCCEx_SPI6_Clock_Source    */

  uint32_t RTCClockSelection;      /*!< Specifies RTC Clock clock source
                                        This parameter can be a value of @ref RCC_RTC_Clock_Source       */

  uint32_t Hrtim1ClockSelection;   /*!< Specifies HRTIM1 Clock clock source
                                        This parameter can be a value of @ref RCCEx_HRTIM1_Clock_Source   */
  uint32_t TIMPresSelection;       /*!< Specifies TIM Clock Prescalers Selection.
                                       This parameter can be a value of @ref RCCEx_TIM_Prescaler_Selection */
}RCC_PeriphCLKInitTypeDef;

typedef struct
{
 __IO uint32_t CR;             /*!< RCC clock control register,                                              Address offset: 0x00  */
 __IO uint32_t ICSCR;          /*!< RCC Internal Clock Sources Calibration Register,                         Address offset: 0x04  */
 __IO uint32_t CRRCR;          /*!< Clock Recovery RC  Register,                                             Address offset: 0x08  */
 uint32_t     RESERVED0;       /*!< Reserved,                                                                Address offset: 0x0C  */
 __IO uint32_t CFGR;           /*!< RCC clock configuration register,                                        Address offset: 0x10  */
 uint32_t     RESERVED1;       /*!< Reserved,                                                                Address offset: 0x14  */
 __IO uint32_t D1CFGR;         /*!< RCC Domain 1 configuration register,                                     Address offset: 0x18  */
 __IO uint32_t D2CFGR;         /*!< RCC Domain 2 configuration register,                                     Address offset: 0x1C  */
 __IO uint32_t D3CFGR;         /*!< RCC Domain 3 configuration register,                                     Address offset: 0x20  */
 uint32_t     RESERVED2;       /*!< Reserved,                                                                Address offset: 0x24  */
 __IO uint32_t PLLCKSELR;      /*!< RCC PLLs Clock Source Selection Register,                                Address offset: 0x28  */
 __IO uint32_t PLLCFGR;        /*!< RCC PLLs  Configuration Register,                                        Address offset: 0x2C  */
 __IO uint32_t PLL1DIVR;       /*!< RCC PLL1 Dividers Configuration Register,                                Address offset: 0x30  */
 __IO uint32_t PLL1FRACR;      /*!< RCC PLL1 Fractional Divider Configuration Register,                      Address offset: 0x34  */
 __IO uint32_t PLL2DIVR;       /*!< RCC PLL2 Dividers Configuration Register,                                Address offset: 0x38  */
 __IO uint32_t PLL2FRACR;      /*!< RCC PLL2 Fractional Divider Configuration Register,                      Address offset: 0x3C  */
 __IO uint32_t PLL3DIVR;       /*!< RCC PLL3 Dividers Configuration Register,                                Address offset: 0x40  */
 __IO uint32_t PLL3FRACR;      /*!< RCC PLL3 Fractional Divider Configuration Register,                      Address offset: 0x44  */
 uint32_t      RESERVED3;      /*!< Reserved,                                                                Address offset: 0x48  */
 __IO uint32_t  D1CCIPR;       /*!< RCC Domain 1 Kernel Clock Configuration Register                         Address offset: 0x4C  */
 __IO uint32_t  D2CCIP1R;      /*!< RCC Domain 2 Kernel Clock Configuration Register                         Address offset: 0x50  */
 __IO uint32_t  D2CCIP2R;      /*!< RCC Domain 2 Kernel Clock Configuration Register                         Address offset: 0x54  */
 __IO uint32_t  D3CCIPR;       /*!< RCC Domain 3 Kernel Clock Configuration Register                         Address offset: 0x58  */
 uint32_t      RESERVED4;      /*!< Reserved,                                                                Address offset: 0x5C  */
 __IO uint32_t  CIER;          /*!< RCC Clock Source Interrupt Enable Register                               Address offset: 0x60  */
 __IO uint32_t  CIFR;          /*!< RCC Clock Source Interrupt Flag Register                                 Address offset: 0x64  */
 __IO uint32_t  CICR;          /*!< RCC Clock Source Interrupt Clear Register                                Address offset: 0x68  */
 uint32_t     RESERVED5;       /*!< Reserved,                                                                Address offset: 0x6C  */
 __IO uint32_t  BDCR;          /*!< RCC Vswitch Backup Domain Control Register,                              Address offset: 0x70  */
 __IO uint32_t  CSR;           /*!< RCC clock control & status register,                                     Address offset: 0x74  */
 uint32_t     RESERVED6;       /*!< Reserved,                                                                Address offset: 0x78  */
 __IO uint32_t AHB3RSTR;       /*!< RCC AHB3 peripheral reset register,                                      Address offset: 0x7C  */
 __IO uint32_t AHB1RSTR;       /*!< RCC AHB1 peripheral reset register,                                      Address offset: 0x80  */
 __IO uint32_t AHB2RSTR;       /*!< RCC AHB2 peripheral reset register,                                      Address offset: 0x84  */
 __IO uint32_t AHB4RSTR;       /*!< RCC AHB4 peripheral reset register,                                      Address offset: 0x88  */
 __IO uint32_t APB3RSTR;       /*!< RCC APB3 peripheral reset register,                                      Address offset: 0x8C  */
 __IO uint32_t APB1LRSTR;      /*!< RCC APB1 peripheral reset Low Word register,                             Address offset: 0x90  */
 __IO uint32_t APB1HRSTR;      /*!< RCC APB1 peripheral reset High Word register,                            Address offset: 0x94  */
 __IO uint32_t APB2RSTR;       /*!< RCC APB2 peripheral reset register,                                      Address offset: 0x98  */
 __IO uint32_t APB4RSTR;       /*!< RCC APB4 peripheral reset register,                                      Address offset: 0x9C  */
 __IO uint32_t GCR;            /*!< RCC RCC Global Control  Register,                                        Address offset: 0xA0  */
 uint32_t     RESERVED7;       /*!< Reserved,                                                                Address offset: 0xA4  */
 __IO uint32_t D3AMR;          /*!< RCC Domain 3 Autonomous Mode Register,                                   Address offset: 0xA8  */
 uint32_t     RESERVED8[9];    /*!< Reserved, 0xAC-0xCC                                                      Address offset: 0xAC  */
 __IO uint32_t RSR;            /*!< RCC Reset status register,                                               Address offset: 0xD0  */
 __IO uint32_t AHB3ENR;        /*!< RCC AHB3 peripheral clock  register,                                     Address offset: 0xD4  */
 __IO uint32_t AHB1ENR;        /*!< RCC AHB1 peripheral clock  register,                                     Address offset: 0xD8  */
 __IO uint32_t AHB2ENR;        /*!< RCC AHB2 peripheral clock  register,                                     Address offset: 0xDC  */
 __IO uint32_t AHB4ENR;        /*!< RCC AHB4 peripheral clock  register,                                     Address offset: 0xE0  */
 __IO uint32_t APB3ENR;        /*!< RCC APB3 peripheral clock  register,                                     Address offset: 0xE4  */
 __IO uint32_t APB1LENR;       /*!< RCC APB1 peripheral clock  Low Word register,                            Address offset: 0xE8  */
 __IO uint32_t APB1HENR;       /*!< RCC APB1 peripheral clock  High Word register,                           Address offset: 0xEC  */
 __IO uint32_t APB2ENR;        /*!< RCC APB2 peripheral clock  register,                                     Address offset: 0xF0  */
 __IO uint32_t APB4ENR;        /*!< RCC APB4 peripheral clock  register,                                     Address offset: 0xF4  */
 uint32_t      RESERVED9;      /*!< Reserved,                                                                Address offset: 0xF8  */
 __IO uint32_t AHB3LPENR;      /*!< RCC AHB3 peripheral sleep clock  register,                               Address offset: 0xFC  */
 __IO uint32_t AHB1LPENR;      /*!< RCC AHB1 peripheral sleep clock  register,                               Address offset: 0x100 */
 __IO uint32_t AHB2LPENR;      /*!< RCC AHB2 peripheral sleep clock  register,                               Address offset: 0x104 */
 __IO uint32_t AHB4LPENR;      /*!< RCC AHB4 peripheral sleep clock  register,                               Address offset: 0x108 */
 __IO uint32_t APB3LPENR;      /*!< RCC APB3 peripheral sleep clock  register,                               Address offset: 0x10C */
 __IO uint32_t APB1LLPENR;     /*!< RCC APB1 peripheral sleep clock  Low Word register,                      Address offset: 0x110 */
 __IO uint32_t APB1HLPENR;     /*!< RCC APB1 peripheral sleep clock  High Word register,                     Address offset: 0x114 */
 __IO uint32_t APB2LPENR;      /*!< RCC APB2 peripheral sleep clock  register,                               Address offset: 0x118 */
 __IO uint32_t APB4LPENR;      /*!< RCC APB4 peripheral sleep clock  register,                               Address offset: 0x11C */
 uint32_t     RESERVED10[4];   /*!< Reserved, 0x120-0x12C                                                    Address offset: 0x120 */

} RCC_TypeDef;

typedef struct
{
  __IM  uint32_t CPUID;                  /*!< Offset: 0x000 (R/ )  CPUID Base Register */
  __IOM uint32_t ICSR;                   /*!< Offset: 0x004 (R/W)  Interrupt Control and State Register */
  __IOM uint32_t VTOR;                   /*!< Offset: 0x008 (R/W)  Vector Table Offset Register */
  __IOM uint32_t AIRCR;                  /*!< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register */
  __IOM uint32_t SCR;                    /*!< Offset: 0x010 (R/W)  System Control Register */
  __IOM uint32_t CCR;                    /*!< Offset: 0x014 (R/W)  Configuration Control Register */
  __IOM uint8_t  SHPR[12U];              /*!< Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15) */
  __IOM uint32_t SHCSR;                  /*!< Offset: 0x024 (R/W)  System Handler Control and State Register */
  __IOM uint32_t CFSR;                   /*!< Offset: 0x028 (R/W)  Configurable Fault Status Register */
  __IOM uint32_t HFSR;                   /*!< Offset: 0x02C (R/W)  HardFault Status Register */
  __IOM uint32_t DFSR;                   /*!< Offset: 0x030 (R/W)  Debug Fault Status Register */
  __IOM uint32_t MMFAR;                  /*!< Offset: 0x034 (R/W)  MemManage Fault Address Register */
  __IOM uint32_t BFAR;                   /*!< Offset: 0x038 (R/W)  BusFault Address Register */
  __IOM uint32_t AFSR;                   /*!< Offset: 0x03C (R/W)  Auxiliary Fault Status Register */
  __IM  uint32_t ID_PFR[2U];             /*!< Offset: 0x040 (R/ )  Processor Feature Register */
  __IM  uint32_t ID_DFR;                 /*!< Offset: 0x048 (R/ )  Debug Feature Register */
  __IM  uint32_t ID_AFR;                 /*!< Offset: 0x04C (R/ )  Auxiliary Feature Register */
  __IM  uint32_t ID_MFR[4U];             /*!< Offset: 0x050 (R/ )  Memory Model Feature Register */
  __IM  uint32_t ID_ISAR[5U];            /*!< Offset: 0x060 (R/ )  Instruction Set Attributes Register */
        uint32_t RESERVED0[1U];
  __IM  uint32_t CLIDR;                  /*!< Offset: 0x078 (R/ )  Cache Level ID register */
  __IM  uint32_t CTR;                    /*!< Offset: 0x07C (R/ )  Cache Type register */
  __IM  uint32_t CCSIDR;                 /*!< Offset: 0x080 (R/ )  Cache Size ID Register */
  __IOM uint32_t CSSELR;                 /*!< Offset: 0x084 (R/W)  Cache Size Selection Register */
  __IOM uint32_t CPACR;                  /*!< Offset: 0x088 (R/W)  Coprocessor Access Control Register */
        uint32_t RESERVED3[93U];
  __OM  uint32_t STIR;                   /*!< Offset: 0x200 ( /W)  Software Triggered Interrupt Register */
        uint32_t RESERVED4[15U];
  __IM  uint32_t MVFR0;                  /*!< Offset: 0x240 (R/ )  Media and VFP Feature Register 0 */
  __IM  uint32_t MVFR1;                  /*!< Offset: 0x244 (R/ )  Media and VFP Feature Register 1 */
  __IM  uint32_t MVFR2;                  /*!< Offset: 0x248 (R/ )  Media and VFP Feature Register 1 */
        uint32_t RESERVED5[1U];
  __OM  uint32_t ICIALLU;                /*!< Offset: 0x250 ( /W)  I-Cache Invalidate All to PoU */
        uint32_t RESERVED6[1U];
  __OM  uint32_t ICIMVAU;                /*!< Offset: 0x258 ( /W)  I-Cache Invalidate by MVA to PoU */
  __OM  uint32_t DCIMVAC;                /*!< Offset: 0x25C ( /W)  D-Cache Invalidate by MVA to PoC */
  __OM  uint32_t DCISW;                  /*!< Offset: 0x260 ( /W)  D-Cache Invalidate by Set-way */
  __OM  uint32_t DCCMVAU;                /*!< Offset: 0x264 ( /W)  D-Cache Clean by MVA to PoU */
  __OM  uint32_t DCCMVAC;                /*!< Offset: 0x268 ( /W)  D-Cache Clean by MVA to PoC */
  __OM  uint32_t DCCSW;                  /*!< Offset: 0x26C ( /W)  D-Cache Clean by Set-way */
  __OM  uint32_t DCCIMVAC;               /*!< Offset: 0x270 ( /W)  D-Cache Clean and Invalidate by MVA to PoC */
  __OM  uint32_t DCCISW;                 /*!< Offset: 0x274 ( /W)  D-Cache Clean and Invalidate by Set-way */
        uint32_t RESERVED7[6U];
  __IOM uint32_t ITCMCR;                 /*!< Offset: 0x290 (R/W)  Instruction Tightly-Coupled Memory Control Register */
  __IOM uint32_t DTCMCR;                 /*!< Offset: 0x294 (R/W)  Data Tightly-Coupled Memory Control Registers */
  __IOM uint32_t AHBPCR;                 /*!< Offset: 0x298 (R/W)  AHBP Control Register */
  __IOM uint32_t CACR;                   /*!< Offset: 0x29C (R/W)  L1 Cache Control Register */
  __IOM uint32_t AHBSCR;                 /*!< Offset: 0x2A0 (R/W)  AHB Slave Control Register */
        uint32_t RESERVED8[1U];
  __IOM uint32_t ABFSR;                  /*!< Offset: 0x2A8 (R/W)  Auxiliary Bus Fault Status Register */
} SCB_Type;

typedef struct
{
  uint32_t Pin;       /*!< Specifies the GPIO pins to be configured.
                           This parameter can be any value of @ref GPIO_pins_define */

  uint32_t Mode;      /*!< Specifies the operating mode for the selected pins.
                           This parameter can be a value of @ref GPIO_mode_define */

  uint32_t Pull;      /*!< Specifies the Pull-up or Pull-Down activation for the selected pins.
                           This parameter can be a value of @ref GPIO_pull_define */

  uint32_t Speed;     /*!< Specifies the speed for the selected pins.
                           This parameter can be a value of @ref GPIO_speed_define */

  uint32_t Alternate;  /*!< Peripheral to be connected to the selected pins.
                            This parameter can be a value of @ref GPIO_Alternate_function_selection */
}GPIO_InitTypeDef;

typedef struct
{
  __IO uint32_t MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
  __IO uint32_t OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
  __IO uint32_t OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
  __IO uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  __IO uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
  __IO uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
  __IO uint16_t BSRRL;    /*!< GPIO port bit set/reset low register,  Address offset: 0x18      */
  __IO uint16_t BSRRH;    /*!< GPIO port bit set/reset high register, Address offset: 0x1A      */
  __IO uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
  __IO uint32_t AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
} GPIO_TypeDef;


#define  GPIO_MODE_INPUT                        ((uint32_t)0x00000000U)   /*!< Input Floating Mode                   */
#define  GPIO_MODE_OUTPUT_PP                    ((uint32_t)0x00000001U)   /*!< Output Push Pull Mode                 */
#define  GPIO_MODE_OUTPUT_OD                    ((uint32_t)0x00000011U)   /*!< Output Open Drain Mode                */
#define  GPIO_MODE_AF_PP                        ((uint32_t)0x00000002U)   /*!< Alternate Function Push Pull Mode     */
#define  GPIO_MODE_AF_OD                        ((uint32_t)0x00000012U)   /*!< Alternate Function Open Drain Mode    */

#define GPIO_MODE             ((uint32_t)0x00000003)
#define ANALOG_MODE           ((uint32_t)0x00000008)
#define EXTI_MODE             ((uint32_t)0x10000000)
#define GPIO_MODE_IT          ((uint32_t)0x00010000)
#define GPIO_MODE_EVT         ((uint32_t)0x00020000)
#define RISING_EDGE           ((uint32_t)0x00100000)
#define FALLING_EDGE          ((uint32_t)0x00200000)
#define GPIO_OUTPUT_TYPE      ((uint32_t)0x00000010)

#define GPIO_MODER_MODER0_Pos            (0U)
#define GPIO_MODER_MODER0_Msk            (0x3U << GPIO_MODER_MODER0_Pos)       /*!< 0x00000003 */
#define GPIO_MODER_MODER0                GPIO_MODER_MODER0_Msk

#define GPIO_OSPEEDER_OSPEEDR0_Pos       (0U)
#define GPIO_OSPEEDER_OSPEEDR0_Msk       (0x3U << GPIO_OSPEEDER_OSPEEDR0_Pos)  /*!< 0x00000003 */
#define GPIO_OSPEEDER_OSPEEDR0           GPIO_OSPEEDER_OSPEEDR0_Msk

#define GPIO_OTYPER_OT_0                 ((uint32_t)0x00000001)

#define GPIO_PUPDR_PUPDR0_Pos            (0U)
#define GPIO_PUPDR_PUPDR0_Msk            (0x3U << GPIO_PUPDR_PUPDR0_Pos)       /*!< 0x00000003 */
#define GPIO_PUPDR_PUPDR0                GPIO_PUPDR_PUPDR0_Msk

#define RCC_AHB4ENR_GPIOIEN_Pos                (8U)
#define RCC_AHB4ENR_GPIOIEN_Msk                (0x1U << RCC_AHB4ENR_GPIOIEN_Pos) /*!< 0x00000100 */
#define RCC_AHB4ENR_GPIOIEN                    RCC_AHB4ENR_GPIOIEN_Msk

#define RCC_AHB4ENR_GPIOBEN_Pos                (1U)
#define RCC_AHB4ENR_GPIOBEN_Msk                (0x1U << RCC_AHB4ENR_GPIOBEN_Pos) /*!< 0x00000002 */
#define RCC_AHB4ENR_GPIOBEN                    RCC_AHB4ENR_GPIOBEN_Msk

#define RCC_AHB4ENR_GPIOFEN_Pos                (5U)
#define RCC_AHB4ENR_GPIOFEN_Msk                (0x1U << RCC_AHB4ENR_GPIOFEN_Pos) /*!< 0x00000020 */
#define RCC_AHB4ENR_GPIOFEN                    RCC_AHB4ENR_GPIOFEN_Msk

#define RCC_AHB4ENR_CRCEN_Pos                  (19U)
#define RCC_AHB4ENR_CRCEN_Msk                  (0x1U << RCC_AHB4ENR_CRCEN_Pos) /*!< 0x00080000 */
#define RCC_AHB4ENR_CRCEN                      RCC_AHB4ENR_CRCEN_Msk

#define RCC_AHB2ENR_D2SRAM1EN_Pos              (29U)
#define RCC_AHB2ENR_D2SRAM1EN_Msk              (0x1U << RCC_AHB2ENR_D2SRAM1EN_Pos) /*!< 0x20000000 */
#define RCC_AHB2ENR_D2SRAM1EN                  RCC_AHB2ENR_D2SRAM1EN_Msk

#define PERIPH_BASE               ((uint32_t)0x40000000)
#define D3_AHB1PERIPH_BASE       (PERIPH_BASE + 0x18020000)

#define GPIOA_BASE            (D3_AHB1PERIPH_BASE + 0x0000)
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)

#define GPIOB_BASE            (D3_AHB1PERIPH_BASE + 0x0400)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)

#define GPIOC_BASE            (D3_AHB1PERIPH_BASE + 0x0800)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)

#define GPIOD_BASE            (D3_AHB1PERIPH_BASE + 0x0C00)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)

#define GPIOI_BASE            (D3_AHB1PERIPH_BASE + 0x2000)
#define GPIOI               ((GPIO_TypeDef *) GPIOI_BASE)

#define GPIOE_BASE            (D3_AHB1PERIPH_BASE + 0x1000)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)

#define GPIOF_BASE            (D3_AHB1PERIPH_BASE + 0x1400)
#define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)

#define GPIOG_BASE            (D3_AHB1PERIPH_BASE + 0x1800)
#define GPIOG               ((GPIO_TypeDef *) GPIOG_BASE)

#define GPIOI_BASE            (D3_AHB1PERIPH_BASE + 0x2000)
#define GPIOI               ((GPIO_TypeDef *) GPIOI_BASE)

#define RCC_CR_HSION_Pos                       (0U)
#define RCC_CR_HSION_Msk                       (0x1U << RCC_CR_HSION_Pos)      /*!< 0x00000001 */
#define RCC_CR_HSION                           RCC_CR_HSION_Msk                /*!< Internal High Speed clock enable */

#define RCC_BASE              (D3_AHB1PERIPH_BASE + 0x4400)
#define RCC                 ((RCC_TypeDef *) RCC_BASE)

/* Memory mapping of Cortex-M4 Hardware */
#define SCS_BASE            (0xE000E000UL)                            /*!< System Control Space Base Address */
#define ITM_BASE            (0xE0000000UL)                            /*!< ITM Base Address */
#define DWT_BASE            (0xE0001000UL)                            /*!< DWT Base Address */
#define TPI_BASE            (0xE0040000UL)                            /*!< TPI Base Address */
#define CoreDebug_BASE      (0xE000EDF0UL)                            /*!< Core Debug Base Address */
#define SysTick_BASE        (SCS_BASE +  0x0010UL)                    /*!< SysTick Base Address */
#define NVIC_BASE           (SCS_BASE +  0x0100UL)                    /*!< NVIC Base Address */
#define SCB_BASE            (SCS_BASE +  0x0D00UL)                    /*!< System Control Block Base Address */

typedef struct
{
  __IOM uint32_t ISER[8U];               /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register */
        uint32_t RESERVED0[24U];
  __IOM uint32_t ICER[8U];               /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register */
        uint32_t RSERVED1[24U];
  __IOM uint32_t ISPR[8U];               /*!< Offset: 0x100 (R/W)  Interrupt Set Pending Register */
        uint32_t RESERVED2[24U];
  __IOM uint32_t ICPR[8U];               /*!< Offset: 0x180 (R/W)  Interrupt Clear Pending Register */
        uint32_t RESERVED3[24U];
  __IOM uint32_t IABR[8U];               /*!< Offset: 0x200 (R/W)  Interrupt Active bit Register */
        uint32_t RESERVED4[56U];
  __IOM uint8_t  IP[240U];               /*!< Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit wide) */
        uint32_t RESERVED5[644U];
  __OM  uint32_t STIR;                   /*!< Offset: 0xE00 ( /W)  Software Trigger Interrupt Register */
}  NVIC_Type;

#define SCnSCB              ((SCnSCB_Type    *)     SCS_BASE      )   /*!< System control Register not in SCB */
#define SCB                 ((SCB_Type       *)     SCB_BASE      )   /*!< SCB configuration struct */
#define SysTick             ((SysTick_Type   *)     SysTick_BASE  )   /*!< SysTick configuration struct */
#define NVIC                ((NVIC_Type      *)     NVIC_BASE     )   /*!< NVIC configuration struct */
#define ITM                 ((ITM_Type       *)     ITM_BASE      )   /*!< ITM configuration struct */
#define DWT                 ((DWT_Type       *)     DWT_BASE      )   /*!< DWT configuration struct */
#define TPI                 ((TPI_Type       *)     TPI_BASE      )   /*!< TPI configuration struct */
#define CoreDebug           ((CoreDebug_Type *)     CoreDebug_BASE)   /*!< Core Debug configuration struct */

#define FLASH_BANK1_BASE          ((uint32_t)0x08000000) //!< Base address of : (up to 1 MB) Flash Bank1 accessible over AXI
#define FLASH_BANK2_BASE          ((uint32_t)0x08100000) //!< Base address of : (up to 1 MB) Flash Bank2 accessible over AXI
#define FLASH_END                 ((uint32_t)0x081FFFFF) //!< FLASH end address

#define RCC_AHB4ENR_GPIOAEN_Pos                (0U)
#define RCC_AHB4ENR_GPIOAEN_Msk                (0x1U << RCC_AHB4ENR_GPIOAEN_Pos) /*!< 0x00000001 */
#define RCC_AHB4ENR_GPIOAEN                    RCC_AHB4ENR_GPIOAEN_Msk

#define RCC_AHB4ENR_GPIOCEN_Pos                (2U)
#define RCC_AHB4ENR_GPIOCEN_Msk                (0x1U << RCC_AHB4ENR_GPIOCEN_Pos) /*!< 0x00000004 */
#define RCC_AHB4ENR_GPIOCEN                    RCC_AHB4ENR_GPIOCEN_Msk

#define RCC_AHB4ENR_GPIODEN_Pos                (3U)
#define RCC_AHB4ENR_GPIODEN_Msk                (0x1U << RCC_AHB4ENR_GPIODEN_Pos) /*!< 0x00000008 */
#define RCC_AHB4ENR_GPIODEN                    RCC_AHB4ENR_GPIODEN_Msk

#define RCC_AHB4ENR_GPIOEEN_Pos                (4U)
#define RCC_AHB4ENR_GPIOEEN_Msk                (0x1U << RCC_AHB4ENR_GPIOEEN_Pos) /*!< 0x00000010 */
#define RCC_AHB4ENR_GPIOEEN                    RCC_AHB4ENR_GPIOEEN_Msk

#define RCC_AHB4ENR_GPIOGEN_Pos                (6U)
#define RCC_AHB4ENR_GPIOGEN_Msk                (0x1U << RCC_AHB4ENR_GPIOGEN_Pos) /*!< 0x00000040 */
#define RCC_AHB4ENR_GPIOGEN                    RCC_AHB4ENR_GPIOGEN_Msk

#define GPIO_PIN_0                 ((uint16_t)0x0001U)  /* Pin 0 selected    */
#define GPIO_PIN_1                 ((uint16_t)0x0002U)  /* Pin 1 selected    */
#define GPIO_PIN_2                 ((uint16_t)0x0004U)  /* Pin 2 selected    */
#define GPIO_PIN_3                 ((uint16_t)0x0008U)  /* Pin 3 selected    */
#define GPIO_PIN_4                 ((uint16_t)0x0010U)  /* Pin 4 selected    */
#define GPIO_PIN_5                 ((uint16_t)0x0020U)  /* Pin 5 selected    */
#define GPIO_PIN_6                 ((uint16_t)0x0040U)  /* Pin 6 selected    */
#define GPIO_PIN_7                 ((uint16_t)0x0080U)  /* Pin 7 selected    */
#define GPIO_PIN_8                 ((uint16_t)0x0100U)  /* Pin 8 selected    */
#define GPIO_PIN_9                 ((uint16_t)0x0200U)  /* Pin 9 selected    */
#define GPIO_PIN_10                ((uint16_t)0x0400U)  /* Pin 10 selected   */
#define GPIO_PIN_11                ((uint16_t)0x0800U)  /* Pin 11 selected   */
#define GPIO_PIN_12                ((uint16_t)0x1000U)  /* Pin 12 selected   */
#define GPIO_PIN_13                ((uint16_t)0x2000U)  /* Pin 13 selected   */
#define GPIO_PIN_14                ((uint16_t)0x4000U)  /* Pin 14 selected   */
#define GPIO_PIN_15                ((uint16_t)0x8000U)  /* Pin 15 selected   */
#define GPIO_PIN_All               ((uint16_t)0xFFFFU)  /* All pins selected */

#define GPIO_PIN_MASK              ((uint32_t)0x0000FFFFU) /* PIN mask for assert test */

#define  GPIO_NOPULL        ((uint32_t)0x00000000U)   /*!< No Pull-up or Pull-down activation  */
#define  GPIO_PULLUP        ((uint32_t)0x00000001U)   /*!< Pull-up activation                  */
#define  GPIO_PULLDOWN      ((uint32_t)0x00000002U)   /*!< Pull-down activation                */

#define  GPIO_SPEED_FREQ_LOW         ((uint32_t)0x00000000U)  /*!< Low speed     */
#define  GPIO_SPEED_FREQ_MEDIUM      ((uint32_t)0x00000001U)  /*!< Medium speed  */
#define  GPIO_SPEED_FREQ_HIGH        ((uint32_t)0x00000002U)  /*!< Fast speed    */
#define  GPIO_SPEED_FREQ_VERY_HIGH   ((uint32_t)0x00000003U)  /*!< High speed    */

//#define SCB                 ((SCB_Type       *)     SCB_BASE      )

#define SCB_AIRCR_VECTKEY_Pos              16U

#define SCB_AIRCR_PRIGROUP_Pos              8U                                            /*!< SCB AIRCR: PRIGROUP Position */
#define SCB_AIRCR_PRIGROUP_Msk             (7UL << SCB_AIRCR_PRIGROUP_Pos)                /*!< SCB AIRCR: PRIGROUP Mask */

#define SCB_AIRCR_SYSRESETREQ_Pos           2U                                            /*!< SCB AIRCR: SYSRESETREQ Position */
#define SCB_AIRCR_SYSRESETREQ_Msk          (1UL << SCB_AIRCR_SYSRESETREQ_Pos)             /*!< SCB AIRCR: SYSRESETREQ Mask */
/**
  \brief   System Reset
  \details Initiates a system reset request to reset the MCU.
 */
__STATIC_INLINE void NVIC_SystemReset(void)
{
  __DSB();                                                          /* Ensure all outstanding memory accesses included
                                                                       buffered write are completed before reset */
  SCB->AIRCR  = (uint32_t)((0x5FAUL << SCB_AIRCR_VECTKEY_Pos)    |
                           (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
                            SCB_AIRCR_SYSRESETREQ_Msk    );         /* Keep priority group unchanged */
  __DSB();                                                          /* Ensure completion of memory access */
}

#define RCC_APB3ENR_LTDCEN_Pos                 (3U)
#define RCC_APB3ENR_LTDCEN_Msk                 (0x1U << RCC_APB3ENR_LTDCEN_Pos) /*!< 0x00000008 */
#define RCC_APB3ENR_LTDCEN                     RCC_APB3ENR_LTDCEN_Msk

#define RCC_AHB3ENR_DMA2DEN_Pos                (4U)
#define RCC_AHB3ENR_DMA2DEN_Msk                (0x1U << RCC_AHB3ENR_DMA2DEN_Pos) /*!< 0x00000010 */
#define RCC_AHB3ENR_DMA2DEN                    RCC_AHB3ENR_DMA2DEN_Msk

// ----------------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------

void gpio_init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init);
unsigned long gpio_clocks_on(void);

#endif
