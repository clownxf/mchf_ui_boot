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

#ifndef __FLASH_H
#define __FLASH_H

#include "main.h"

//#define FLASH_BASE_ADDR      (uint32_t)(FLASH_BASE)
#define FLASH_END_ADDR       (uint32_t)(0x081FFFFF)

/* Base address of the Flash sectors Bank 1 */
#define ADDR_FLASH_SECTOR_0_BANK1     ((uint32_t)0x08000000) /* Base @ of Sector 0, 128 Kbytes */
#define ADDR_FLASH_SECTOR_1_BANK1     ((uint32_t)0x08020000) /* Base @ of Sector 1, 128 Kbytes */
#define ADDR_FLASH_SECTOR_2_BANK1     ((uint32_t)0x08040000) /* Base @ of Sector 2, 128 Kbytes */
#define ADDR_FLASH_SECTOR_3_BANK1     ((uint32_t)0x08060000) /* Base @ of Sector 3, 128 Kbytes */
#define ADDR_FLASH_SECTOR_4_BANK1     ((uint32_t)0x08080000) /* Base @ of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5_BANK1     ((uint32_t)0x080A0000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6_BANK1     ((uint32_t)0x080C0000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7_BANK1     ((uint32_t)0x080E0000) /* Base @ of Sector 7, 128 Kbytes */

/* Base address of the Flash sectors Bank 2 */
#define ADDR_FLASH_SECTOR_0_BANK2     ((uint32_t)0x08100000) /* Base @ of Sector 0, 128 Kbytes */
#define ADDR_FLASH_SECTOR_1_BANK2     ((uint32_t)0x08120000) /* Base @ of Sector 1, 128 Kbytes */
#define ADDR_FLASH_SECTOR_2_BANK2     ((uint32_t)0x08140000) /* Base @ of Sector 2, 128 Kbytes */
#define ADDR_FLASH_SECTOR_3_BANK2     ((uint32_t)0x08160000) /* Base @ of Sector 3, 128 Kbytes */
#define ADDR_FLASH_SECTOR_4_BANK2     ((uint32_t)0x08180000) /* Base @ of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5_BANK2     ((uint32_t)0x081A0000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6_BANK2     ((uint32_t)0x081C0000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7_BANK2     ((uint32_t)0x081E0000) /* Base @ of Sector 7, 128 Kbytes */

#define FLASH_SECTOR_0     ((uint32_t)0) /*!< Sector Number 0   */
#define FLASH_SECTOR_1     ((uint32_t)1) /*!< Sector Number 1   */
#define FLASH_SECTOR_2     ((uint32_t)2) /*!< Sector Number 2   */
#define FLASH_SECTOR_3     ((uint32_t)3) /*!< Sector Number 3   */
#define FLASH_SECTOR_4     ((uint32_t)4) /*!< Sector Number 4   */
#define FLASH_SECTOR_5     ((uint32_t)5) /*!< Sector Number 5   */
#define FLASH_SECTOR_6     ((uint32_t)6) /*!< Sector Number 6   */
#define FLASH_SECTOR_7     ((uint32_t)7) /*!< Sector Number 7   */

typedef struct
{
  __IO uint32_t ACR;             /*!< FLASH access control register,                           Address offset: 0x00 */
  __IO uint32_t KEYR1;           /*!< Flash Key Register for bank1,                            Address offset: 0x04 */
  __IO uint32_t OPTKEYR;         /*!< Flash Option Key Register,                                Address offset: 0x08 */
  __IO uint32_t CR1;             /*!< Flash Control Register for bank1,                        Address offset: 0x0C */
  __IO uint32_t SR1;             /*!< Flash Status Register for bank1,                         Address offset: 0x10 */
  __IO uint32_t CCR1;            /*!< Flash Control Register for bank1,                        Address offset: 0x14 */
  __IO uint32_t OPTCR;           /*!< Flash Option Control Register,                            Address offset: 0x18 */
  __IO uint32_t OPTSR_CUR;       /*!< Flash Option Status Current Register,                     Address offset: 0x1C */
  __IO uint32_t OPTSR_PRG;       /*!< Flash Option Status Current Register,                     Address offset: 0x20 */
  __IO uint32_t OPTCCR;          /*!< Flash Option Clear Control Register,                      Address offset: 0x24 */
  __IO uint32_t PRAR_CUR1;       /*!< Flash Current Protection Address Register for bank1,     Address offset: 0x28 */
  __IO uint32_t PRAR_PRG1;       /*!< Flash Protection Address to Program Register for bank1,  Address offset: 0x2C */
  __IO uint32_t SCAR_CUR1;       /*!< Flash Current Secure Address Register for bank1,         Address offset: 0x30 */
  __IO uint32_t SCAR_PRG1;       /*!< Flash Secure Address Register for bank1,                 Address offset: 0x34 */
  __IO uint32_t WPSN_CUR1;       /*!< Flash Current Write Protection Register on bank1,        Address offset: 0x38 */
  __IO uint32_t WPSN_PRG1;       /*!< Flash Write Protection to Program Register on bank1,     Address offset: 0x3C */
  __IO uint32_t BOOT_CUR;        /*!< Flash Current Boot Address for Pelican Core Register,     Address offset: 0x40 */
  __IO uint32_t BOOT_PRG;        /*!< Flash Boot Address to Program for Pelican Core Register,  Address offset: 0x44 */
  uint32_t      RESERVED0[2];    /*!< Reserved, 0x48 to 0x4C                                                        */
  __IO uint32_t CRCCR1;          /*!< Flash CRC Control register For Bank1 Register ,          Address offset: 0x50 */
  __IO uint32_t CRCSADD1;        /*!< Flash CRC Start Address Register for Bank1 ,             Address offset: 0x54 */
  __IO uint32_t CRCEADD1;        /*!< Flash CRC End Address Register for Bank1 ,               Address offset: 0x58 */
  __IO uint32_t CRCDATA;         /*!< Flash CRC Data Register for Bank1 ,                      Address offset: 0x5C */
  __IO uint32_t ECC_FA1;         /*!< Flash ECC Fail Address For Bank1 Register ,              Address offset: 0x60 */
  uint32_t      RESERVED1[40];   /*!< Reserved, 0x64 to 0x100                                                       */
  __IO uint32_t KEYR2;           /*!< Flash Key Register for bank2,                           Address offset: 0x104 */
  uint32_t      RESERVED2;       /*!< Reserved, 0x108                                                               */
  __IO uint32_t CR2;             /*!< Flash Control Register for bank2,                       Address offset: 0x10C */
  __IO uint32_t SR2;             /*!< Flash Status Register for bank2,                        Address offset: 0x110 */
  __IO uint32_t CCR2;            /*!< Flash Status Register for bank2,                        Address offset: 0x114 */
  uint32_t      RESERVED3[4];    /*!< Reserved, 0x118 to 0x124                                                      */
  __IO uint32_t PRAR_CUR2;       /*!< Flash Current Protection Address Register for bank2,    Address offset: 0x128 */
  __IO uint32_t PRAR_PRG2;       /*!< Flash Protection Address to Program Register for bank2, Address offset: 0x12C */
  __IO uint32_t SCAR_CUR2;       /*!< Flash Current Secure Address Register for bank2,        Address offset: 0x130 */
  __IO uint32_t SCAR_PRG2;       /*!< Flash Secure Address Register for bank2,                Address offset: 0x134 */
  __IO uint32_t WPSN_CUR2;       /*!< Flash Current Write Protection Register on bank2,       Address offset: 0x138 */
  __IO uint32_t WPSN_PRG2;       /*!< Flash Write Protection to Program Register on bank2,    Address offset: 0x13C */
  uint32_t      RESERVED4[4];    /*!< Reserved, 0x140 to 0x14C                                                      */
  __IO uint32_t CRCCR2;          /*!< Flash CRC Control register For Bank2 Register ,         Address offset: 0x150 */
  __IO uint32_t CRCSADD2;        /*!< Flash CRC Start Address Register for Bank2 ,            Address offset: 0x154 */
  __IO uint32_t CRCEADD2;        /*!< Flash CRC End Address Register for Bank2 ,              Address offset: 0x158 */
  __IO uint32_t CRCDATA2;        /*!< Flash CRC Data Register for Bank2 ,                     Address offset: 0x15C */
  __IO uint32_t ECC_FA2;         /*!< Flash ECC Fail Address For Bank2 Register ,             Address offset: 0x160 */
} FLASH_TypeDef;

typedef struct
{
  uint32_t TypeErase;   /*!< Mass erase or sector Erase.
                             This parameter can be a value of @ref FLASHEx_Type_Erase */

  uint32_t Banks;       /*!< Select banks to erase when Mass erase is enabled.
                             This parameter must be a value of @ref FLASHEx_Banks */

  uint32_t Sector;      /*!< Initial FLASH sector to erase when Mass erase is disabled
                             This parameter must be a value of @ref FLASH_Sectors */

  uint32_t NbSectors;   /*!< Number of sectors to be erased.
                             This parameter must be a value between 1 and (max number of sectors - value of Initial sector)*/

  uint32_t VoltageRange;/*!< The device voltage range which defines the erase parallelism
                             This parameter must be a value of @ref FLASHEx_Voltage_Range */

} FLASH_EraseInitTypeDef;

#define PERIPH_BASE               ((uint32_t)0x40000000)
#define D1_AHB1PERIPH_BASE       (PERIPH_BASE + 0x12000000)
#define FLASH_R_BASE          	(D1_AHB1PERIPH_BASE + 0x2000)
#define FLASH               	((FLASH_TypeDef *) FLASH_R_BASE)

#define FLASH_CR_LOCK_Pos                    (0U)
#define FLASH_CR_LOCK_Msk                    (0x1U << FLASH_CR_LOCK_Pos)       /*!< 0x00000001 */
#define FLASH_CR_LOCK                        FLASH_CR_LOCK_Msk

#define FLASH_KEY1               ((uint32_t)0x45670123U)
#define FLASH_KEY2               ((uint32_t)0xCDEF89ABU)

#define FLASH_TYPEERASE_SECTORS         ((uint32_t)0x00U)  /*!< Sectors erase only          */
#define FLASH_TYPEERASE_MASSERASE       ((uint32_t)0x01U)  /*!< Flash Mass erase activation */

#define FLASH_CR_PSIZE_Pos                   (4U)
#define FLASH_CR_PSIZE_Msk                   (0x3U << FLASH_CR_PSIZE_Pos)      /*!< 0x00000030 */
#define FLASH_CR_PSIZE                       FLASH_CR_PSIZE_Msk
#define FLASH_CR_PSIZE_0                     (0x1U << FLASH_CR_PSIZE_Pos)      /*!< 0x00000010 */
#define FLASH_CR_PSIZE_1                     (0x2U << FLASH_CR_PSIZE_Pos)      /*!< 0x00000020 */

#define FLASH_VOLTAGE_RANGE_1        ((uint32_t)0x00U)             /*!< Flash program/erase by 8 bits    */
#define FLASH_VOLTAGE_RANGE_2        ((uint32_t)FLASH_CR_PSIZE_0)  /*!< Flash program/erase by 16 bits   */
#define FLASH_VOLTAGE_RANGE_3        ((uint32_t)FLASH_CR_PSIZE_1)  /*!< Flash program/erase by 32 bits   */
#define FLASH_VOLTAGE_RANGE_4        ((uint32_t)FLASH_CR_PSIZE)    /*!< Flash program/erase by 64 bits   */

#define FLASH_BANK_1                       ((uint32_t)0x01U)                          /*!< Bank 1   */
#define FLASH_BANK_2                       ((uint32_t)0x02U)                          /*!< Bank 2   */
#define FLASH_BANK_BOTH                    ((uint32_t)(FLASH_BANK_1 | FLASH_BANK_2)) /*!< Bank1 and Bank2  */

#define FLASH_TIMEOUT_VALUE       	((uint32_t)50000U)/* 50 s */

#define FLASH_SR_BSY_Pos                     (0U)
#define FLASH_SR_BSY_Msk                     (0x1U << FLASH_SR_BSY_Pos)        /*!< 0x00000001 */
#define FLASH_SR_BSY                         FLASH_SR_BSY_Msk
#define FLASH_SR_WBNE_Pos                    (1U)
#define FLASH_SR_WBNE_Msk                    (0x1U << FLASH_SR_WBNE_Pos)       /*!< 0x00000002 */
#define FLASH_SR_WBNE                        FLASH_SR_WBNE_Msk
#define FLASH_SR_QW_Pos                      (2U)
#define FLASH_SR_QW_Msk                      (0x1U << FLASH_SR_QW_Pos)         /*!< 0x00000004 */
#define FLASH_SR_QW                          FLASH_SR_QW_Msk

#define FLASH_SR_EOP_Pos                     (16U)
#define FLASH_SR_EOP_Msk                     (0x1U << FLASH_SR_EOP_Pos)        /*!< 0x00010000 */
#define FLASH_SR_EOP                         FLASH_SR_EOP_Msk
#define FLASH_SR_WRPERR_Pos                  (17U)
#define FLASH_SR_WRPERR_Msk                  (0x1U << FLASH_SR_WRPERR_Pos)     /*!< 0x00020000 */
#define FLASH_SR_WRPERR                      FLASH_SR_WRPERR_Msk
#define FLASH_SR_PGSERR_Pos                  (18U)
#define FLASH_SR_PGSERR_Msk                  (0x1U << FLASH_SR_PGSERR_Pos)     /*!< 0x00040000 */
#define FLASH_SR_PGSERR                      FLASH_SR_PGSERR_Msk
#define FLASH_SR_STRBERR_Pos                 (19U)
#define FLASH_SR_STRBERR_Msk                 (0x1U << FLASH_SR_STRBERR_Pos)    /*!< 0x00080000 */
#define FLASH_SR_STRBERR                     FLASH_SR_STRBERR_Msk
#define FLASH_SR_INCERR_Pos                  (21U)
#define FLASH_SR_INCERR_Msk                  (0x1U << FLASH_SR_INCERR_Pos)     /*!< 0x00200000 */
#define FLASH_SR_INCERR                      FLASH_SR_INCERR_Msk
#define FLASH_SR_OPERR_Pos                   (22U)
#define FLASH_SR_OPERR_Msk                   (0x1U << FLASH_SR_OPERR_Pos)      /*!< 0x00400000 */
#define FLASH_SR_OPERR                       FLASH_SR_OPERR_Msk

#define FLASH_SR_RDPERR_Pos                  (23U)
#define FLASH_SR_RDPERR_Msk                  (0x1U << FLASH_SR_RDPERR_Pos)     /*!< 0x00800000 */
#define FLASH_SR_RDPERR                      FLASH_SR_RDPERR_Msk
#define FLASH_SR_RDSERR_Pos                  (24U)
#define FLASH_SR_RDSERR_Msk                  (0x1U << FLASH_SR_RDSERR_Pos)     /*!< 0x01000000 */
#define FLASH_SR_RDSERR                      FLASH_SR_RDSERR_Msk

#define FLASH_SR_SNECCERR_Pos                (25U)
#define FLASH_SR_SNECCERR_Msk                (0x1U << FLASH_SR_SNECCERR_Pos)   /*!< 0x02000000 */
#define FLASH_SR_SNECCERR                    FLASH_SR_SNECCERR_Msk
#define FLASH_SR_DBECCERR_Pos                (26U)
#define FLASH_SR_DBECCERR_Msk                (0x1U << FLASH_SR_DBECCERR_Pos)   /*!< 0x04000000 */
#define FLASH_SR_DBECCERR                    FLASH_SR_DBECCERR_Msk

#define FLASH_SR_CRC_BUSY_Pos                (3U)
#define FLASH_SR_CRC_BUSY_Msk                (0x1U << FLASH_SR_CRC_BUSY_Pos)   /*!< 0x00000008 */
#define FLASH_SR_CRC_BUSY                    FLASH_SR_CRC_BUSY_Msk

#define FLASH_SR_CRCEND_Pos                  (27U)
#define FLASH_SR_CRCEND_Msk                  (0x1U << FLASH_SR_CRCEND_Pos)     /*!< 0x08000000 */
#define FLASH_SR_CRCEND                      FLASH_SR_CRCEND_Msk


#define FLASH_FLAG_BSY_BANK1               FLASH_SR_BSY             /*!< FLASH Bank 1 Busy flag */
#define FLASH_FLAG_WBNE_BANK1              FLASH_SR_WBNE            /*!< Waiting for Data to Write on Bank 1 flag */
#define FLASH_FLAG_QW_BANK1                FLASH_SR_QW              /*!< Write Waiting in Operation Queue on Bank 1 flag */
#define FLASH_FLAG_CRC_BUSY_BANK1          FLASH_SR_CRC_BUSY        /*!< CRC module is working on Bank 1 flag */
#define FLASH_FLAG_EOP_BANK1               FLASH_SR_EOP             /*!< End Of Program on Bank 1 flag */
#define FLASH_FLAG_WRPERR_BANK1            FLASH_SR_WRPERR          /*!< Write Protection Error on Bank 1 flag */
#define FLASH_FLAG_PGSERR_BANK1            FLASH_SR_PGSERR          /*!< Program Sequence Error on Bank 1 flag */
#define FLASH_FLAG_STRBER_BANK1R           FLASH_SR_STRBERR         /*!< strobe Error on Bank 1 flag */
#define FLASH_FLAG_INCERR_BANK1            FLASH_SR_INCERR          /*!< Inconsistency Error on Bank 1 flag */
#define FLASH_FLAG_OPERR_BANK1             FLASH_SR_OPERR           /*!< Operation Error on Bank 1 flag */
#define FLASH_FLAG_RDPERR_BANK1            FLASH_SR_RDPERR          /*!< Read Protection Error on Bank 1 flag */
#define FLASH_FLAG_RDSERR_BANK1            FLASH_SR_RDSERR          /*!< Read Secured Error on Bank 1 flag */
#define FLASH_FLAG_SNECCE_BANK1RR          FLASH_SR_SNECCERR        /*!< Single ECC Error Correction on Bank 1 flag */
#define FLASH_FLAG_DBECCE_BANK1RR          FLASH_SR_DBECCERR        /*!< Double Detection ECC Error on Bank 1 flag */
#define FLASH_FLAG_CRCEND_BANK1            FLASH_SR_CRCEND          /*!< CRC module completes on bank Bank 1 flag */

/*******************  Bits definition for FLASH_OPTCR register  *******************/
#define FLASH_OPTCR_OPTLOCK_Pos              (0U)
#define FLASH_OPTCR_OPTLOCK_Msk              (0x1U << FLASH_OPTCR_OPTLOCK_Pos) /*!< 0x00000001 */
#define FLASH_OPTCR_OPTLOCK                  FLASH_OPTCR_OPTLOCK_Msk
#define FLASH_OPTCR_OPTSTART_Pos             (1U)
#define FLASH_OPTCR_OPTSTART_Msk             (0x1U << FLASH_OPTCR_OPTSTART_Pos) /*!< 0x00000002 */
#define FLASH_OPTCR_OPTSTART                 FLASH_OPTCR_OPTSTART_Msk
#define FLASH_OPTCR_MER_Pos                  (3U)
#define FLASH_OPTCR_MER_Msk                  (0x1U << FLASH_OPTCR_MER_Pos)     /*!< 0x00000008 */
#define FLASH_OPTCR_MER                      FLASH_OPTCR_MER_Msk
#define FLASH_OPTCR_OPTCHANGEERRIE_Pos       (30U)
#define FLASH_OPTCR_OPTCHANGEERRIE_Msk       (0x1U << FLASH_OPTCR_OPTCHANGEERRIE_Pos) /*!< 0x40000000 */
#define FLASH_OPTCR_OPTCHANGEERRIE           FLASH_OPTCR_OPTCHANGEERRIE_Msk
#define FLASH_OPTCR_SWAP_BANK_Pos            (31U)
#define FLASH_OPTCR_SWAP_BANK_Msk            (0x1U << FLASH_OPTCR_SWAP_BANK_Pos) /*!< 0x80000000 */
#define FLASH_OPTCR_SWAP_BANK                FLASH_OPTCR_SWAP_BANK_Msk

#define FLASH_FLAG_BSY_BANK2               (FLASH_SR_BSY      | 0x80000000U)        /*!< FLASH Bank 2 Busy flag */
#define FLASH_FLAG_WBNE_BANK2              (FLASH_SR_WBNE     | 0x80000000U)        /*!< Waiting for Data to Write on Bank 2 flag */
#define FLASH_FLAG_QW_BANK2                (FLASH_SR_QW       | 0x80000000U)        /*!< Write Waiting in Operation Queue on Bank 2 flag */
#define FLASH_FLAG_CRC_BUSY_BANK2          (FLASH_SR_CRC_BUSY | 0x80000000U)        /*!< CRC module is working on Bank 2 flag */
#define FLASH_FLAG_EOP_BANK2               (FLASH_SR_EOP      | 0x80000000U)        /*!< End Of Program on Bank 2 flag */
#define FLASH_FLAG_WRPERR_BANK2            (FLASH_SR_WRPERR   | 0x80000000U)        /*!< Write Protection Error on Bank 2 flag */
#define FLASH_FLAG_PGSERR_BANK2            (FLASH_SR_PGSERR   | 0x80000000U)        /*!< Program Sequence Error on Bank 2 flag */
#define FLASH_FLAG_STRBER_BANK2R           (FLASH_SR_STRBERR  | 0x80000000U)        /*!< Strobe Error on Bank 2 flag */
#define FLASH_FLAG_INCERR_BANK2            (FLASH_SR_INCERR   | 0x80000000U)        /*!< Inconsistency Error on Bank 2 flag */
#define FLASH_FLAG_OPERR_BANK2             (FLASH_SR_OPERR    | 0x80000000U)        /*!< Operation Error on Bank 2 flag */
#define FLASH_FLAG_RDPERR_BANK2            (FLASH_SR_RDPERR   | 0x80000000U)        /*!< Read Protection Error on Bank 2 flag */
#define FLASH_FLAG_RDSERR_BANK2            (FLASH_SR_RDSERR   | 0x80000000U)        /*!< Read Secured Error on Bank 2 flag */
#define FLASH_FLAG_SNECCE_BANK2RR          (FLASH_SR_SNECCERR | 0x80000000U)        /*!< Single ECC Error Correction on Bank 2 flag */
#define FLASH_FLAG_DBECCE_BANK2RR          (FLASH_SR_DBECCERR | 0x80000000U)        /*!< Double Detection ECC Error on Bank 2 flag */
#define FLASH_FLAG_CRCEND_BANK2            (FLASH_SR_CRCEND   | 0x80000000U)        /*!< CRC module completes on bank Bank 2 flag */

#define FLASH_FLAG_ALL_ERRORS_BANK1       (FLASH_FLAG_WRPERR_BANK1  | FLASH_FLAG_PGSERR_BANK1   | \
                                           FLASH_FLAG_STRBER_BANK1R | FLASH_FLAG_INCERR_BANK1   | \
                                           FLASH_FLAG_OPERR_BANK1   | FLASH_FLAG_RDPERR_BANK1   | \
                                           FLASH_FLAG_RDSERR_BANK1  | FLASH_FLAG_SNECCE_BANK1RR | \
                                           FLASH_FLAG_DBECCE_BANK1RR)

#define FLASH_FLAG_ALL_ERRORS_BANK2       (FLASH_FLAG_WRPERR_BANK2  | FLASH_FLAG_PGSERR_BANK2   | \
                                           FLASH_FLAG_STRBER_BANK2R | FLASH_FLAG_INCERR_BANK2   | \
                                           FLASH_FLAG_OPERR_BANK2   | FLASH_FLAG_RDPERR_BANK2   | \
                                           FLASH_FLAG_RDSERR_BANK2  | FLASH_FLAG_SNECCE_BANK2RR | \
                                           FLASH_FLAG_DBECCE_BANK2RR)

#define __HAL_FLASH_CLEAR_FLAG_BANK1(__FLAG__)    WRITE_REG(FLASH->CCR1, (__FLAG__))

#define __HAL_FLASH_CLEAR_FLAG_BANK2(__FLAG__)    WRITE_REG(FLASH->CCR2, ((__FLAG__) & 0x7FFFFFFF))

#define __HAL_FLASH_CLEAR_FLAG(__FLAG__)         (IS_FLASH_FLAG_BANK1(__FLAG__) ?  __HAL_FLASH_CLEAR_FLAG_BANK1(__FLAG__) : \
                                                   __HAL_FLASH_CLEAR_FLAG_BANK2(__FLAG__))

#define FLASH_FLAG_ALL_BANK1              (FLASH_FLAG_BSY_BANK1      | FLASH_FLAG_WBNE_BANK1      | \
                                           FLASH_FLAG_QW_BANK1       | FLASH_FLAG_CRC_BUSY_BANK1 | \
                                           FLASH_FLAG_EOP_BANK1      | FLASH_FLAG_CRCEND_BANK1   | \
                                           FLASH_FLAG_ALL_ERRORS_BANK1)

#define IS_FLASH_FLAG_BANK1(FLAG)          (((FLAG) & FLASH_FLAG_ALL_BANK1) == (FLAG))

#define IS_FLASH_FLAG_BANK2(FLAG)          (((FLAG) & FLASH_FLAG_ALL_BANK2) == (FLAG))

#define __HAL_FLASH_GET_FLAG_BANK1(__FLAG__)          (READ_BIT(FLASH->SR1, (__FLAG__)) == (__FLAG__))

#define __HAL_FLASH_GET_FLAG_BANK2(__FLAG__)          (READ_BIT(FLASH->SR2, ((__FLAG__) & 0x7FFFFFFF)) == (((__FLAG__) & 0x7FFFFFFF)))

#define __HAL_FLASH_GET_FLAG(__FLAG__)          (IS_FLASH_FLAG_BANK1(__FLAG__) ?  __HAL_FLASH_GET_FLAG_BANK1(__FLAG__) : \
                                                 __HAL_FLASH_GET_FLAG_BANK2(__FLAG__))


#define FLASH_CR_SER_Pos                     (2U)
#define FLASH_CR_SER_Msk                     (0x1U << FLASH_CR_SER_Pos)        /*!< 0x00000004 */
#define FLASH_CR_SER                         FLASH_CR_SER_Msk

#define FLASH_CR_SNB_Pos                     (8U)
#define FLASH_CR_SNB_Msk                     (0x7U << FLASH_CR_SNB_Pos)        /*!< 0x00000700 */
#define FLASH_CR_SNB                         FLASH_CR_SNB_Msk

#define FLASH_CR_START_Pos                   (7U)
#define FLASH_CR_START_Msk                   (0x1U << FLASH_CR_START_Pos)      /*!< 0x00000080 */
#define FLASH_CR_START                       FLASH_CR_START_Msk

#define FLASH_CR_PG_Pos                      (1U)
#define FLASH_CR_PG_Msk                      (0x1U << FLASH_CR_PG_Pos)         /*!< 0x00000002 */
#define FLASH_CR_PG                          FLASH_CR_PG_Msk

#define FLASH_SIZE                         0x200000  /* 2MB */
#define FLASH_BANK_SIZE                    (FLASH_SIZE >> 1)   /* 1MB */

#define FLASH_SECTOR_SIZE 0x00020000 /* 128 KB */

#define FLASH_BANK1_BASE          ((uint32_t)0x08000000) /*!< Base address of : (up to 1 MB) Flash Bank1 accessible over AXI                          */
#define FLASH_BANK2_BASE          ((uint32_t)0x08100000) /*!< Base address of : (up to 1 MB) Flash Bank2 accessible over AXI                          */
#define FLASH_END                 ((uint32_t)0x081FFFFF) /*!< FLASH end address                                                                       */

#define IS_FLASH_PROGRAM_ADDRESS_BANK1(ADDRESS) (((ADDRESS) >= FLASH_BANK1_BASE) && ((ADDRESS) < (FLASH_BANK1_BASE + FLASH_BANK_SIZE) ))
#define IS_FLASH_PROGRAM_ADDRESS_BANK2(ADDRESS) (((ADDRESS) >= FLASH_BANK2_BASE ) && ((ADDRESS) < (FLASH_BANK2_BASE + FLASH_BANK_SIZE) ))
#define IS_FLASH_PROGRAM_ADDRESS(ADDRESS)  (IS_FLASH_PROGRAM_ADDRESS_BANK1(ADDRESS) || IS_FLASH_PROGRAM_ADDRESS_BANK2(ADDRESS))

#define FLASH_TYPEPROGRAM_FLASHWORD  ((uint32_t)0x03U)  /*!< Program a flash word (256-bit) at a specified address */

// -----------------------------------------------------------------------------------

uchar flash_unlock(void);
uchar flash_lock(void);

uchar flash_erase_firmware(void);
//uchar flash_program(uint32_t TypeProgram, uint32_t FlashAddress, uint64_t DataAddress);

uchar flash_write_block(uchar *block,ulong addr);

#endif
