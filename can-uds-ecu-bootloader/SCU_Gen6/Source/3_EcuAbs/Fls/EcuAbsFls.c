#include "Definition.h"
#include "Rte_EcuAbs.h"
#include "FlsLoader.h"
#include "EcuAbsFls.h"
#include "EcuAbsWdg.h"

void EA_initFls(void)
{
    FlsLoader_Init(NULL);
}

void EA_erasePflashBlock(uint8 pflashArea, P_uint8 errorResult)
{
    Mcal_DisableAllInterrupts();
    EA_wdgTriggerUpdateSlow();
    if(pflashArea == PFLASH_AREA_APPICATION_PRIMARY)
    {
        *errorResult = (uint8_t)FlsLoader_Erase(FLSLOADER_PFLASH0_START_ADDRESS + FLSLOADER_PF_S23, 2u);
    }
    else if(pflashArea == PFLASH_AREA_APPICATION_SECONDARY)
    {
        *errorResult = (uint8_t)FlsLoader_Erase(FLSLOADER_PFLASH0_START_ADDRESS + FLSLOADER_PF_S25, 2u);
    }
    else
    {
        *errorResult = !FALSE;
    }
    Mcal_EnableAllInterrupts();
}

void EA_writePflash(uint32 flashAddress, uint32 dataSize, P_uint8 data, P_uint8 errorResult)
{
    EA_wdgTriggerUpdateSlow();
    *errorResult = (uint8_t)FlsLoader_Write(flashAddress, dataSize,data);
}

// source에서 Target으로 복사, 결과 저장 errorResult
void EA_copyPflashSector(uint8 pflashAreaTarget, uint8 pflashAreaSource, P_uint8 errorResult)
{
    // 복사할 내용이 있는 곳 -> second, 복사하여 붙이는 곳 -> pri
    if((pflashAreaTarget== PFLASH_AREA_APPICATION_PRIMARY) && (pflashAreaSource== PFLASH_AREA_APPICATION_SECONDARY))
    {
        // 내용을 write 하려면 pri 지워야 함 -> 지움
        EA_erasePflashBlock(PFLASH_AREA_APPICATION_PRIMARY, errorResult);
        if(*errorResult== FALSE)
        {   // 오류 없으면 app 실행 주소에 데이터 쓰기
            EA_writePflash(MEMORY_ADDRESS_APPLICATION, MEMORY_SIZE_APPLICATION +
                    MEMORY_DATA_VALID_PATTERN_LENGTH, (P_uint8)(MEMORY_ADDRESS_APPLICATION+MEMORY_SECONDARY_OFFSET), errorResult);
        }
    }
    else
    {
        *errorResult= !FALSE;
    }
}
