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

#ifndef __GPIO_H
#define __GPIO_H

#include "main.h"

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


void gpio_init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init);
unsigned long gpio_clocks_on(void);

#endif
