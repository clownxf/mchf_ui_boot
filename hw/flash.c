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

#include "flash.h"
#include "misc.h"
#include "gpio.h"

static FLASH_EraseInitTypeDef EraseInitStruct;

uchar flash_unlock(void)
{
#ifndef DISSABLE_FLASH_SUPPORT
  if((READ_BIT(FLASH->CR1, FLASH_CR_LOCK) != RESET) && (READ_BIT(FLASH->CR2, FLASH_CR_LOCK) != RESET))
  {
    // Authorise the FLASH A Registers access
    WRITE_REG(FLASH->KEYR1, FLASH_KEY1);
    WRITE_REG(FLASH->KEYR1, FLASH_KEY2);
    // Authorise the FLASH B Registers access
    WRITE_REG(FLASH->KEYR2, FLASH_KEY1);
    WRITE_REG(FLASH->KEYR2, FLASH_KEY2);
  }
  else
    return 1;
#endif
  return 0;
}

uchar flash_lock(void)
{
#ifndef DISSABLE_FLASH_SUPPORT
  // Set the LOCK Bit to lock the FLASH A Registers access
  SET_BIT(FLASH->CR1, FLASH_CR_LOCK);
  // Set the LOCK Bit to lock the FLASH B Registers access
  SET_BIT(FLASH->CR2, FLASH_CR_LOCK);
#endif
  return 0;
}

// Wait for the FLASH operation to complete by polling on BUSY flag to be reset.
//     Even if the FLASH operation fails, the BUSY flag will be reset and an error
//     flag will be set
uchar flash_wait_last_operation(uint32_t Timeout, uint32_t Bank)
{
  uint32_t bsyflag, errorflag = 0;
#ifndef DISSABLE_FLASH_SUPPORT
  if(Bank == FLASH_BANK_1)
  {
    bsyflag = FLASH_FLAG_BSY_BANK1 | FLASH_FLAG_QW_BANK1;

    if((FLASH->OPTCR & FLASH_OPTCR_SWAP_BANK) == 0)
    {
      bsyflag |= FLASH_FLAG_WBNE_BANK1;
    }
    else
    {
      bsyflag |= FLASH_FLAG_WBNE_BANK2;
    }
  }
  else
  {
    bsyflag = FLASH_FLAG_BSY_BANK2 | FLASH_FLAG_QW_BANK2;

    if((FLASH->OPTCR & FLASH_OPTCR_SWAP_BANK) == 0)
    {
      bsyflag |= FLASH_FLAG_WBNE_BANK2;
    }
    else
    {
      bsyflag |= FLASH_FLAG_WBNE_BANK1;
    }
  }

  while(__HAL_FLASH_GET_FLAG(bsyflag))
  {
	  Timeout--;
	  if(Timeout == 0)
		  return 3;

	  misc_cpu_dependent_delay(0x7FFFFFF);
  }

  if((Bank == FLASH_BANK_1) && ((FLASH->SR1 & FLASH_FLAG_ALL_ERRORS_BANK1) != RESET))
  {
    errorflag = FLASH_FLAG_ALL_ERRORS_BANK1;
  }
  else if((Bank == FLASH_BANK_2) && ((FLASH->SR2 & FLASH_FLAG_ALL_ERRORS_BANK2 & 0x7FFFFFFF) != RESET))
  {
    errorflag = FLASH_FLAG_ALL_ERRORS_BANK2;
  }

  if(errorflag != 0)
  {
    // Save the error code
    //FLASH_SetErrorCode(Bank);

    // Clear error programming flags
    __HAL_FLASH_CLEAR_FLAG(errorflag);

    return 1;
  }
#endif
  // If there is an error flag set
  return 0;
}

void flash_erase_sector(uint32_t Sector, uint32_t Banks, uint32_t VoltageRange)
{
#ifndef DISSABLE_FLASH_SUPPORT
  if((Banks & FLASH_BANK_1) == FLASH_BANK_1)
  {
    // reset Program/erase VoltageRange for Bank1
    FLASH->CR1 &= ~(FLASH_CR_PSIZE | FLASH_CR_SNB);

    FLASH->CR1 |= (FLASH_CR_SER | VoltageRange | (Sector << POSITION_VAL(FLASH_CR_SNB)));

    FLASH->CR1 |= FLASH_CR_START;
  }
  //
  if((Banks & FLASH_BANK_2) == FLASH_BANK_2)
  {
    // reset Program/erase VoltageRange for Bank2
    FLASH->CR2 &= ~(FLASH_CR_PSIZE | FLASH_CR_SNB);

    FLASH->CR2 |= (FLASH_CR_SER | VoltageRange  | (Sector << POSITION_VAL(FLASH_CR_SNB)));

    FLASH->CR2 |= FLASH_CR_START;
  }
#endif
}

//uchar flag = 0;
uchar flash_erase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *SectorError)
{
  uchar 	status = 0;
  uint32_t 	index = 0;
#ifndef DISSABLE_FLASH_SUPPORT
  // Wait for last operation to be completed
  if((pEraseInit->Banks & FLASH_BANK_1) == FLASH_BANK_1)
  {
    status = flash_wait_last_operation((uint32_t)FLASH_TIMEOUT_VALUE, FLASH_BANK_1);
  }

  if((pEraseInit->Banks & FLASH_BANK_2) == FLASH_BANK_2)
  {
    status |= flash_wait_last_operation((uint32_t)FLASH_TIMEOUT_VALUE, FLASH_BANK_2);
  }

  // Erase by sector by sector to be done
  for(index = pEraseInit->Sector; index < (pEraseInit->NbSectors + pEraseInit->Sector); index++)
  {
  	   flash_erase_sector(index, pEraseInit->Banks, pEraseInit->VoltageRange);

        if((pEraseInit->Banks & FLASH_BANK_1) == FLASH_BANK_1)
        {
          // Wait for last operation to be completed
          status = flash_wait_last_operation((uint32_t)FLASH_TIMEOUT_VALUE, FLASH_BANK_1);

          // If the erase operation is completed, disable the SER Bit
          FLASH->CR1 &= (~(FLASH_CR_SER | FLASH_CR_SNB));
        }

        if((pEraseInit->Banks & FLASH_BANK_2) == FLASH_BANK_2)
        {
          // Wait for last operation to be completed
          status = flash_wait_last_operation((uint32_t)FLASH_TIMEOUT_VALUE, FLASH_BANK_2);

          // If the erase operation is completed, disable the SER Bit
          FLASH->CR2 &= (~(FLASH_CR_SER | FLASH_CR_SNB));
        }

        if(status != 0)
        {
          // In case of error, stop erase procedure and return the faulty sector
          *SectorError = index;
          break;
     }
#if 0
        // Blink while erase (test)
        if(flag)
        	GPIOD->BSRRL = ((uint16_t)0x0080U);
        else
        	GPIOD->BSRRH = ((uint16_t)0x0080U);

        flag = !flag;
#endif
  }
#endif
  return status;
}

uchar flash_program(uint32_t TypeProgram, uint32_t FlashAddress, uint64_t DataAddress)
{
      uchar status = 1;
      __IO uint64_t *dest_addr = (__IO uint64_t *)FlashAddress;
      __IO uint64_t *src_addr = (__IO uint64_t*)((uint32_t)DataAddress);

      uint32_t bank;
      uint8_t row_index = 4;

#ifndef DISSABLE_FLASH_SUPPORT
      if(IS_FLASH_PROGRAM_ADDRESS_BANK1(FlashAddress))
      {
        bank = FLASH_BANK_1;
      }
      else
      {
        bank = FLASH_BANK_2;
      }

      /* Wait for last operation to be completed */
      status = flash_wait_last_operation((uint32_t)FLASH_TIMEOUT_VALUE, bank);

      if(status == 0)
      {
        if(bank == FLASH_BANK_1)
        {
          /* Clear bank 1 pending flags (if any) */
          __HAL_FLASH_CLEAR_FLAG_BANK1(FLASH_FLAG_EOP_BANK1 | FLASH_FLAG_QW_BANK1 | FLASH_FLAG_WBNE_BANK1 | FLASH_FLAG_ALL_ERRORS_BANK1);

          /* Set PG bit */
          SET_BIT(FLASH->CR1, FLASH_CR_PG);
        }
        else
        {
          /* Clear bank 2 pending flags (if any) */
          __HAL_FLASH_CLEAR_FLAG_BANK2(FLASH_FLAG_EOP_BANK2 | FLASH_FLAG_QW_BANK2 | FLASH_FLAG_WBNE_BANK2 | FLASH_FLAG_ALL_ERRORS_BANK2);

          /* Set PG bit */
          SET_BIT(FLASH->CR2, FLASH_CR_PG);
        }

        /* Program the 256 bits flash word */
        do
        {
          *dest_addr++ = *src_addr++;
        } while (--row_index != 0);

        __DSB();

        /* Wait for last operation to be completed */
        status = flash_wait_last_operation((uint32_t)FLASH_TIMEOUT_VALUE, bank);

        if(bank == FLASH_BANK_1)
        {
          /* Check FLASH End of Operation flag  */
          if (__HAL_FLASH_GET_FLAG_BANK1(FLASH_FLAG_EOP_BANK1))
          {
            /* Clear FLASH End of Operation pending bit */
            __HAL_FLASH_CLEAR_FLAG_BANK1(FLASH_FLAG_EOP_BANK1);
          }

          /* If the program operation is completed, disable the PG*/
          CLEAR_BIT(FLASH->CR1, FLASH_CR_PG);
        }
        else
        {
          /* Check FLASH End of Operation flag  */
          if (__HAL_FLASH_GET_FLAG_BANK2(FLASH_FLAG_EOP_BANK2))
          {
            /* Clear FLASH End of Operation pending bit */
            __HAL_FLASH_CLEAR_FLAG_BANK2(FLASH_FLAG_EOP_BANK2);
          }

          /* If the program operation is completed, disable the PG */
          CLEAR_BIT(FLASH->CR2, FLASH_CR_PG);
        }
      }

      /* Process Unlocked */
      //__HAL_UNLOCK(&pFlash);
#endif
      return status;
}

// In theory we can pass firmware size and div by 128k
// sectors to reduce erase time, but too much hassle, so
// just nuke everything !
uchar flash_erase_firmware(void)
{
	uint32_t SECTORError = 0;

	// All sectors, except 0(bootloader)
	EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Banks         = FLASH_BANK_1;
	EraseInitStruct.Sector        = FLASH_SECTOR_1;
	EraseInitStruct.NbSectors     = 7;

	// Nuke bank one
	if (flash_erase(&EraseInitStruct, &SECTORError) != 0)
	{
		return 1;
	}

	// All sectors
	EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Banks         = FLASH_BANK_2;
	EraseInitStruct.Sector        = FLASH_SECTOR_0;
	EraseInitStruct.NbSectors     = 8;

	// Nuke bank two
	if (flash_erase(&EraseInitStruct, &SECTORError) != 0)
	{
		return 1;
	}

	return 0;
}

uint32_t Address = ADDR_FLASH_SECTOR_1_BANK1;	// - local increment

uchar flash_write_block(uchar *block,ulong addr)
{
	//FLASH_Status status = FLASH_COMPLETE;
	ulong i,j;

	//uint64_t FlashWordA[4];

	uchar 	 data_block[32];

	//uint32_t Address = addr;		// - via passed address

	for(i = 0; i < 16; i++)
	{
		for(j = 0; j < 32; j++)
		{
			data_block[j] = *block;
			block++;
		}

		// ToDo: check result...
		flash_program(FLASH_TYPEPROGRAM_FLASHWORD, Address, (uint64_t)((uint32_t)data_block));

		// Next
		Address += 0x20;
	}

	return 0;
}
