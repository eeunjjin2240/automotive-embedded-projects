#include "Rte_BswDcm.h"
#include "Definition.h"
#include "UdsNrcList.h"
#include "Utility.h"
#include "KISA_SHA256.h"

#define DIAG_BUFFER_SIZE 4096u

#define FRAME_TYPE_SINGLE 0u
#define FRAME_TYPE_FIRST        1u
#define FRAME_TYPE_CONSECUTIVE  2u
#define FRAME_TYPE_FLOW_CONTROL 3u

#define REF_VALUE_FLOW_STATUS_CTS   0u
#define REF_VALUE_BLOCK_SIZE        0u
#define REF_VALUE_ST_MIN            0u

#define TIME_DIAG_P2_CAN_SERVER		TIME_50MS
#define TIME_DIAG_P2_MUL_CAN_SERVER	TIME_5S
#define TIME_DIAG_S3_SERVER         TIME_5S
#define REF_VALUE_FILLER_BYTE		0xAAu
#define MAX_NUMBER_OF_BLOCK_LENGTH  (512u+2u)

#define SID_DiagSessionControl				0x10u
#define SID_EcuReset						0x11u
#define SID_SecurityAccess					0x27u
#define SID_CommunicationControl			0x28u
#define SID_RoutineControl					0x31u
#define SID_WriteDataByIdentifier			0x2Eu
#define SID_ReadDataByIdentifier			0x22u
#define SID_RequestDownload                 0x34u
#define SID_TransferData                    0x36u
#define SID_RequestTransferExit             0x37u

#define SFID_DiagnosticSessionControl_DefaultSession						0x01u
#define SFID_DiagnosticSessionControl_ProgrammingSession		            0x02u
#define SFID_DiagnosticSessionControl_ExtendedSession						0x03u
#define SFID_EcuReset_HardReset 											0x01u
#define SFID_SecurityAccess_RequestSeed										0x01u
#define SFID_SecurityAccess_SendKey											0x02u
#define SFID_CommunicationControl_enableRxAndTx								0x00u
#define SFID_CommunicationControl_disableRxAndTx							0x03u
#define SFID_RoutineControl_Start						0x01u
#define DID_VariantCode 0xF101u
#define RID_AliveCountIncrease					0xF000u
#define RID_AliveCountDecrease					0xF001u
#define RID_CheckProgrammingDependencies        0x0001u
#define RID_EraseMemory                         0x0002u
#define RCOR_EraseMemory_App                    0xF001u

#define CommunicationType_normalCommunicationMessages		0x01u

typedef struct {
	ST_DiagStatus stDiagStatus;
    uint8 isServiceRequest;
    uint8 isPhysical;
    uint8 diagDataBufferRx[DIAG_BUFFER_SIZE];
    uint8 diagDataBufferTx[MAX_CAN_MSG_DLC];
    uint16 diagDataLengthRx;
    uint8 diagDataLengthTx;
    uint32 timeS3ServerStart;
	uint16 firstFrameDataLength;
    uint32 downloadMemoryAddress;
    uint32 downloadMemorySize;
    uint32 downloadDataLength;
    uint8 downloadBlockSequence;

    SHA256_INFO signInfo;
    uint8 signResult[MEMORY_SIZE_SIGNATURE];
    uint8 signKey[KEY_SIZE_SIGNATURE];

}ST_BswDcmData;

static ST_BswDcmData stBswDcmData;

static uint8 diagnosticSessionControl(void);
static uint8 ecuReset(void);
static uint8 securityAccess(void);
static uint8 communicationControl(void);
static uint8 routineControl(void);
static uint8 writeDataByIdentifier(void);
static uint8 readDataByIdentifier(void);
static uint8 requestDownload(void);
static uint8 transferData(void);
static uint8 requestTransferExit(void);

void REcbBswDcm_initialize(void)
{
    stBswDcmData.stDiagStatus.currentSession = DIAG_SESSION_PROGRAMMING;
    memset(stBswDcmData.signKey, 0x11, KEY_SIZE_SIGNATURE);  // KEY값 0x11
}

void REtBswDcm_bswDcmMain(void)
{
	uint8 serviceId;
	uint8 errorResult = FALSE;
	if(stBswDcmData.isServiceRequest == TRUE)
	{
		stBswDcmData.isServiceRequest = FALSE;
		serviceId = stBswDcmData.diagDataBufferRx[0];
		if(serviceId == SID_DiagSessionControl)
		{
			errorResult = diagnosticSessionControl();
		}
		else if(serviceId == SID_EcuReset)
		{
			errorResult = ecuReset();
		}
		else if(serviceId == SID_SecurityAccess)
		{
			errorResult = securityAccess();
		}
		else if(serviceId == SID_CommunicationControl)
		{
			errorResult = communicationControl();
		}
		else if(serviceId == SID_RoutineControl)
		{
			errorResult = routineControl();
		}
		else if(serviceId == SID_WriteDataByIdentifier)
		{
			errorResult = writeDataByIdentifier();
		}
		else if(serviceId == SID_ReadDataByIdentifier)
		{
			errorResult = readDataByIdentifier();
		}
        else if(serviceId == SID_RequestDownload)
        {
            errorResult = requestDownload();
        }
        else if(serviceId == SID_TransferData)
        {
            errorResult = transferData();
        }
        else if(serviceId == SID_RequestTransferExit)
        {
            errorResult = requestTransferExit();
        }
		else
		{
			errorResult = NRC_ServiceNotSupported;
		}

		if(errorResult != FALSE)
		{
			stBswDcmData.diagDataLengthTx = 3u;
			stBswDcmData.diagDataBufferTx[0] = (FRAME_TYPE_SINGLE << 4u) | stBswDcmData.diagDataLengthTx;
			stBswDcmData.diagDataBufferTx[1] = 0x7Fu;
			stBswDcmData.diagDataBufferTx[2] = stBswDcmData.diagDataBufferRx[0];
			stBswDcmData.diagDataBufferTx[3] = errorResult;
		}

		for(uint8 i=stBswDcmData.diagDataLengthTx+1u; i<TX_MSG_LEN_PHY_RES; i++)
		{
			stBswDcmData.diagDataBufferTx[i] = REF_VALUE_FILLER_BYTE;
		}
		Rte_Call_BswDcm_rEcuAbsCan_canSend(TX_MSG_IDX_PHY_RES, stBswDcmData.diagDataBufferTx);
	}
	if(UT_getTimeEcuAlive1ms() > (stBswDcmData.timeS3ServerStart + TIME_DIAG_S3_SERVER))
	{
	    stBswDcmData.stDiagStatus.currentSession = DIAG_SESSION_DEFAULT;
	}
}

void REoiBswDcm_pDcmCmd_processDiagRequest(uint8 isPhysical, P_uint8 diagReq, uint8 length)
{
    uint8_t frameType;
    stBswDcmData.timeS3ServerStart = UT_getTimeEcuAlive1ms();

    frameType = diagReq[0] >> 4u;
    if (frameType == FRAME_TYPE_SINGLE)
    {
        stBswDcmData.isPhysical = isPhysical;
        stBswDcmData.diagDataLengthRx = (diagReq[0] & 0x0Fu);
        (void)memcpy(stBswDcmData.diagDataBufferRx, &diagReq[1], stBswDcmData.diagDataLengthRx);
        stBswDcmData.isServiceRequest = TRUE;
    }
    else if (frameType == FRAME_TYPE_FIRST)
    {
        stBswDcmData.firstFrameDataLength = (((uint16)diagReq[0] & 0x000Fu) << 8u) | diagReq[1];
        stBswDcmData.diagDataLengthRx = length - 2u;
        (void)memcpy(stBswDcmData.diagDataBufferRx, &diagReq[2], stBswDcmData.diagDataLengthRx);
        // Send FlowControl
        (void) memset(stBswDcmData.diagDataBufferTx, REF_VALUE_FILLER_BYTE, TX_MSG_LEN_PHY_RES);
        stBswDcmData.diagDataBufferTx[0] = (FRAME_TYPE_FLOW_CONTROL << 4u) | REF_VALUE_FLOW_STATUS_CTS;
        stBswDcmData.diagDataBufferTx[1] = REF_VALUE_BLOCK_SIZE;
        stBswDcmData.diagDataBufferTx[2] = REF_VALUE_ST_MIN;
        Rte_Call_BswDcm_rEcuAbsCan_canSend(TX_MSG_IDX_PHY_RES, stBswDcmData.diagDataBufferTx);
    }
    else if (frameType == FRAME_TYPE_CONSECUTIVE)
    {
        uint16 remainDataLength = stBswDcmData.firstFrameDataLength - stBswDcmData.diagDataLengthRx;
        if(remainDataLength > (length-1u))
        {
            (void)memcpy(&stBswDcmData.diagDataBufferRx[stBswDcmData.diagDataLengthRx], &diagReq[1], length-1u);
            stBswDcmData.diagDataLengthRx += (length-1u);
        }
        else
        {
            (void)memcpy(&stBswDcmData.diagDataBufferRx[stBswDcmData.diagDataLengthRx], &diagReq[1], remainDataLength);
            stBswDcmData.diagDataLengthRx += remainDataLength;
            stBswDcmData.isServiceRequest = TRUE;
        }
    }
    else
    {
        (void) 0;
    }
}

void REoiBswDcm_pDcmCmd_getDiagStatus(P_stDiagStatus pstDiagStatus)
{
	(void)memcpy(pstDiagStatus, &stBswDcmData.stDiagStatus, sizeof(ST_DiagStatus));
}

static uint8 diagnosticSessionControl(void)
{
	uint8 errorResult = FALSE;
	uint8 subFunctionId = stBswDcmData.diagDataBufferRx[1] & 0x7Fu;
	if(stBswDcmData.diagDataLengthRx != 2u)
	{
		errorResult = NRC_IncorrectMessageLengthOrInvalidFormat;
	}
	else if(subFunctionId == SFID_DiagnosticSessionControl_DefaultSession)
	{
		stBswDcmData.stDiagStatus.currentSession = DIAG_SESSION_DEFAULT;
	}
	else if(subFunctionId == SFID_DiagnosticSessionControl_ProgrammingSession)
	{
		stBswDcmData.stDiagStatus.currentSession = DIAG_SESSION_PROGRAMMING;
	}
	else if(subFunctionId == SFID_DiagnosticSessionControl_ExtendedSession)
	{
		stBswDcmData.stDiagStatus.currentSession = DIAG_SESSION_EXTENDED;
	}
	else
	{
		errorResult = NRC_SubFunctionNotSupported;
	}
	if(errorResult == FALSE)
	{
		stBswDcmData.diagDataLengthTx = 6u;
		stBswDcmData.diagDataBufferTx[0] = (FRAME_TYPE_SINGLE << 4u) | stBswDcmData.diagDataLengthTx;
		stBswDcmData.diagDataBufferTx[1] = SID_DiagSessionControl + 0x40u;
		stBswDcmData.diagDataBufferTx[2] = subFunctionId;
		stBswDcmData.diagDataBufferTx[3] = (TIME_DIAG_P2_CAN_SERVER & 0xFF00u) >> 8u;		/* High Byte : TIME_DIAG_P2_CAN_SERVER */
		stBswDcmData.diagDataBufferTx[4] = (TIME_DIAG_P2_CAN_SERVER & 0xFFu);				/* Low Byte : TIME_DIAG_P2_CAN_SERVER */
		stBswDcmData.diagDataBufferTx[5] = (uint8)(((TIME_DIAG_P2_MUL_CAN_SERVER/10u) & 0xFF00u) >> 8u);	/* High Byte : P2*CAN_SERVER */
		stBswDcmData.diagDataBufferTx[6] = ((TIME_DIAG_P2_MUL_CAN_SERVER/10u) & 0xFFu);			/* Low Byte : P2*CAN_SERVER */
	}
	return errorResult;
}

static uint8 ecuReset(void)
{
	uint8 subFunctionId = stBswDcmData.diagDataBufferRx[1] & 0x7Fu;
	uint8 errorResult = FALSE;

	if (stBswDcmData.diagDataLengthRx != 2u)
	{
		errorResult = NRC_IncorrectMessageLengthOrInvalidFormat;
	}
	else if(subFunctionId == SFID_EcuReset_HardReset)
	{
		stBswDcmData.stDiagStatus.isEcuReset = TRUE;
	}
	else
	{
		errorResult = NRC_SubFunctionNotSupported;
	}
	if(errorResult == FALSE)
	{
		stBswDcmData.diagDataLengthTx = 2u;
		stBswDcmData.diagDataBufferTx[0] = (FRAME_TYPE_SINGLE << 4u) | stBswDcmData.diagDataLengthTx;
		stBswDcmData.diagDataBufferTx[1] = SID_EcuReset + 0x40u;
		stBswDcmData.diagDataBufferTx[2] = subFunctionId;
	}
	return errorResult;
}

static uint8 securityAccess(void)
{
    uint8 subFunctionId = stBswDcmData.diagDataBufferRx[1];
    uint16 seed = 0x1234;
    uint16 key;
    uint8 errorResult = FALSE;
    stBswDcmData.stDiagStatus.isSecurityUnlock = FALSE;
    if(stBswDcmData.stDiagStatus.currentSession == DIAG_SESSION_DEFAULT)
    {
        errorResult = NRC_ServiceNotSupportedInActiveSession;
    }
    else if(subFunctionId == SFID_SecurityAccess_RequestSeed)
    {
        if(stBswDcmData.diagDataLengthRx != 2u)
        {
            errorResult = NRC_IncorrectMessageLengthOrInvalidFormat;
        }
        else
        {
            stBswDcmData.diagDataLengthTx = 4u;
            stBswDcmData.diagDataBufferTx[0] = (FRAME_TYPE_SINGLE << 4u) | stBswDcmData.diagDataLengthTx;
            stBswDcmData.diagDataBufferTx[1] = SID_SecurityAccess + 0x40u;
            stBswDcmData.diagDataBufferTx[2] = subFunctionId;
            stBswDcmData.diagDataBufferTx[3] = (uint8)((seed & 0xFF00u) >> 8u);
            stBswDcmData.diagDataBufferTx[4] = (uint8)(seed & 0xFFu);
        }
    }
    else if(subFunctionId == SFID_SecurityAccess_SendKey)
    {
        if(stBswDcmData.diagDataLengthRx != 4u)
        {
            errorResult = NRC_IncorrectMessageLengthOrInvalidFormat;
        }
        else
        {
            key = ((uint16)stBswDcmData.diagDataBufferRx[2] << 8u) | stBswDcmData.diagDataBufferRx[3];
            if(key == 0x5678u)
            {
                stBswDcmData.diagDataLengthTx = 2u;
                stBswDcmData.diagDataBufferTx[0] = (FRAME_TYPE_SINGLE << 4u) | stBswDcmData.diagDataLengthTx;
                stBswDcmData.diagDataBufferTx[1] = SID_SecurityAccess + 0x40u;
                stBswDcmData.diagDataBufferTx[2] = subFunctionId;
                stBswDcmData.stDiagStatus.isSecurityUnlock = TRUE;
            }
            else
            {
                errorResult = NRC_InvalidKey;
            }
        }
    }
    else
    {
        errorResult = NRC_SubFunctionNotSupported;
    }
    return errorResult;
}

static uint8 communicationControl(void)
{
	uint8 subFunctionId = stBswDcmData.diagDataBufferRx[1] & 0x7Fu;
	uint8 errorResult = FALSE;
	uint8 communicationType = stBswDcmData.diagDataBufferRx[2];

	if(stBswDcmData.diagDataLengthRx != 3u)
	{
		errorResult = NRC_IncorrectMessageLengthOrInvalidFormat;
	}
	else if(stBswDcmData.stDiagStatus.currentSession != DIAG_SESSION_EXTENDED)
	{
		errorResult = NRC_SubFunctionNotSupportedInActiveSession;
	}
	else if(communicationType != CommunicationType_normalCommunicationMessages)
	{
		errorResult = NRC_RequestOutRange;
	}
	else if(subFunctionId == SFID_CommunicationControl_enableRxAndTx)
	{
		stBswDcmData.stDiagStatus.isCommunicationDisable = FALSE;
		stBswDcmData.diagDataLengthTx = 2u;
		stBswDcmData.diagDataBufferTx[0] = (FRAME_TYPE_SINGLE << 4u) | stBswDcmData.diagDataLengthTx;
		stBswDcmData.diagDataBufferTx[1] = SID_CommunicationControl + 0x40u;
		stBswDcmData.diagDataBufferTx[2] = subFunctionId;
	}
	else if(subFunctionId == SFID_CommunicationControl_disableRxAndTx)
	{
		stBswDcmData.stDiagStatus.isCommunicationDisable = TRUE;
		stBswDcmData.diagDataLengthTx = 2u;
		stBswDcmData.diagDataBufferTx[0] = (FRAME_TYPE_SINGLE << 4u) | stBswDcmData.diagDataLengthTx;
		stBswDcmData.diagDataBufferTx[1] = SID_CommunicationControl + 0x40u;
		stBswDcmData.diagDataBufferTx[2] = subFunctionId;
	}
	else
	{
		errorResult = NRC_SubFunctionNotSupported;
	}
	return errorResult;
}

static uint8 routineControl(void)
{
	uint8 errorResult = FALSE;   // 오류 없음
	uint8 subFunctionId = stBswDcmData.diagDataBufferRx[1];  // 서브펑션id 가져오기
	uint16_t routineIdentifier = ((uint16_t)stBswDcmData.diagDataBufferRx[2] << 8) | (uint16_t)stBswDcmData.diagDataBufferRx[3];  // RID 2바이트
    uint16 routineControlOptionRecord;

	if(stBswDcmData.diagDataLengthRx < 4u)  // SID+SF+PID = 4바이트 기본
	{
		errorResult = NRC_IncorrectMessageLengthOrInvalidFormat;  // 사이즈 안맞음
	}
	else if(subFunctionId != SFID_RoutineControl_Start) // START (0x01) 외에 다 오류 처리
	{
	    errorResult = NRC_SubFunctionNotSupported;
	}
	else if (routineIdentifier == RID_AliveCountIncrease)
	{
	    stBswDcmData.stDiagStatus.isAliveCountDecrease = FALSE;
	}
	else if (routineIdentifier == RID_AliveCountDecrease)
	{
	    stBswDcmData.stDiagStatus.isAliveCountDecrease = TRUE;
	}
	else if(routineIdentifier == RID_EraseMemory)  // 메모리 지우기면, 추가 파라미터 확인하고 플래시 erase 수행
	{
	    routineControlOptionRecord = ((uint16_t)stBswDcmData.diagDataBufferRx[4] << 8) | (uint16_t)stBswDcmData.diagDataBufferRx[5];  // 옵션레코드 2바이트 읽기
	    if(routineControlOptionRecord == RCOR_EraseMemory_App)  // 옵션이 app erase이면
	    {
	        // 백업영역 pflash erase 요청
	        Rte_Call_BswDcm_rEcuAbsFls_erasePflashBlock(PFLASH_AREA_APPICATION_SECONDARY, &errorResult);
	    }
	    else
	    {
	        // 그 옵션 아니면 신경 안씀
	        errorResult = NRC_RequestOutRange;
	    }
	}
	// 앱 다운로드 중 reset 전 마지막 단계
	else if(routineIdentifier == RID_CheckProgrammingDependencies)
	{
	    // RPO2 추가 - SHA 해시 알고리즘
//	    SHA256_Init(&stBswDcmData.signInfo);
//	    SHA256_Process(&stBswDcmData.signInfo, stBswDcmData.signKey, KEY_SIZE_SIGNATURE);
//	    // app1 업로드 된 주소 시작부터 패턴 전까지로 sha 계산
//        SHA256_Process(&stBswDcmData.signInfo, (void*)(MEMORY_ADDRESS_APPLICATION+MEMORY_SECONDARY_OFFSET), MEMORY_SIZE_APPLICATION-MEMORY_SIZE_SIGNATURE);
//        SHA256_Close(&stBswDcmData.signInfo, stBswDcmData.signResult);

        // 패턴 주소부터 패턴 길이 만큼 비교
        if(memcmp(stBswDcmData.signResult, (void*)(MEMORY_ADDRESS_APPLICATION+MEMORY_SECONDARY_OFFSET+MEMORY_SIZE_APPLICATION-MEMORY_SIZE_SIGNATURE), MEMORY_SIZE_SIGNATURE) == 0)
        {
            Rte_Call_BswDcm_rEcuAbsFls_copyPflashSector(PFLASH_AREA_APPICATION_PRIMARY, PFLASH_AREA_APPICATION_SECONDARY, &errorResult);
            if(errorResult != FALSE) {
                errorResult = NRC_GeneralProgrammingFailure;
            }
        }
        else {
            errorResult = NRC_GeneralProgrammingFailure;
        }
	}
	else
	{
		errorResult = NRC_RequestOutRange;
	}

	if(errorResult == FALSE)
	{
		stBswDcmData.diagDataLengthTx = 4u;
		stBswDcmData.diagDataBufferTx[0] = (FRAME_TYPE_SINGLE << 4u) | stBswDcmData.diagDataLengthTx;
		stBswDcmData.diagDataBufferTx[1] = SID_RoutineControl + 0x40u;
		stBswDcmData.diagDataBufferTx[2] = subFunctionId;
		stBswDcmData.diagDataBufferTx[3] = (uint8)((routineIdentifier & 0xFF00) >> 8u);
		stBswDcmData.diagDataBufferTx[4] = (uint8)(routineIdentifier & 0xFFu);
	}
	return errorResult;
}

static uint8 writeDataByIdentifier(void)
{
	uint8 variantCode[DATA_LEN_VariantCode] = {0u,};
	uint8 errorResult = FALSE;
	uint16 dataIdentifier = ((uint16_t)stBswDcmData.diagDataBufferRx[1] << 8) | (uint16_t)stBswDcmData.diagDataBufferRx[2];

	if(stBswDcmData.stDiagStatus.currentSession != DIAG_SESSION_EXTENDED)
	{
	    errorResult = NRC_ServiceNotSupportedInActiveSession;
	}
	else if(stBswDcmData.stDiagStatus.isSecurityUnlock == FALSE)
    {
        errorResult = NRC_SecurityAccessDenied;
    }
	else if(dataIdentifier == DID_VariantCode)
	{
		if(stBswDcmData.diagDataLengthRx != 7u)
		{
			errorResult = NRC_IncorrectMessageLengthOrInvalidFormat;
		}
		else
		{
			variantCode[0] = stBswDcmData.diagDataBufferRx[3];
			variantCode[1] = stBswDcmData.diagDataBufferRx[4];
			variantCode[2] = stBswDcmData.diagDataBufferRx[5];
			variantCode[3] = stBswDcmData.diagDataBufferRx[6];
			Rte_Call_BswDcm_rEcuAbsNvm_writeNvmData(DATA_ID_VariantCode, variantCode);
			stBswDcmData.diagDataLengthTx = 3u;
			stBswDcmData.diagDataBufferTx[0] = (FRAME_TYPE_SINGLE << 4u) | stBswDcmData.diagDataLengthTx;
			stBswDcmData.diagDataBufferTx[1] = SID_WriteDataByIdentifier + 0x40u;
			stBswDcmData.diagDataBufferTx[2] = (uint8)((dataIdentifier & 0xFF00) >> 8u);
			stBswDcmData.diagDataBufferTx[3] = (uint8)(dataIdentifier & 0xFFu);
		}
	}
	else
	{
		errorResult = NRC_RequestOutRange;
	}
	return errorResult;
}

static uint8 readDataByIdentifier(void)
{
	uint8 variantCode[DATA_LEN_VariantCode] = {0u,};
	uint8 errorResult = FALSE;
	uint16 dataIdentifier = ((uint16_t)stBswDcmData.diagDataBufferRx[1] << 8) | (uint16_t)stBswDcmData.diagDataBufferRx[2];

	if(stBswDcmData.diagDataLengthRx != 3u)
	{
		errorResult = NRC_IncorrectMessageLengthOrInvalidFormat;
	}
	else if(dataIdentifier == DID_VariantCode)
	{
		Rte_Call_BswDcm_rEcuAbsNvm_readNvmData(DATA_ID_VariantCode, variantCode);
		stBswDcmData.diagDataLengthTx = 7u;
		stBswDcmData.diagDataBufferTx[0] = (FRAME_TYPE_SINGLE << 4u) | stBswDcmData.diagDataLengthTx;
		stBswDcmData.diagDataBufferTx[1] = SID_ReadDataByIdentifier + 0x40u;
		stBswDcmData.diagDataBufferTx[2] = (uint8)((dataIdentifier & 0xFF00) >> 8u);
		stBswDcmData.diagDataBufferTx[3] = (uint8)(dataIdentifier & 0xFFu);
		stBswDcmData.diagDataBufferTx[4] = variantCode[0];
		stBswDcmData.diagDataBufferTx[5] = variantCode[1];
		stBswDcmData.diagDataBufferTx[6] = variantCode[2];
		stBswDcmData.diagDataBufferTx[7] = variantCode[3];
	}
	else
	{
		errorResult = NRC_RequestOutRange;
	}
	return errorResult;
}

static uint8 requestDownload(void)
{
    uint8 errorResult;
    uint8 dataFormatIdentifier = stBswDcmData.diagDataBufferRx[1];
    uint8 compressionMethod = (dataFormatIdentifier & 0xF0u) >> 4u;
    uint8 encryptingMethod = dataFormatIdentifier & 0x0Fu;
    uint8 addressAndLengthFormatIdentifier = stBswDcmData.diagDataBufferRx[2];
    uint32 lengthOfDataSize = (addressAndLengthFormatIdentifier & 0xF0u) >> 4u;
    uint32 lengthOfAddress = addressAndLengthFormatIdentifier & 0x0Fu;
    uint32 memoryAddress;
    uint32 memorySize;

    if(stBswDcmData.diagDataLengthRx != 11u)
    {// sid(1) + data_formatIdentifier(1)+ALFI(1)+ADDRESS(4)+Size(4) = 11
        errorResult = NRC_IncorrectMessageLengthOrInvalidFormat;
    }
    else if(stBswDcmData.stDiagStatus.isSecurityUnlock == FALSE)
    {
        // security 채제 안되면 다운로드 권한 없음
        errorResult = NRC_SecurityAccessDenied;
    }
    else if((compressionMethod != 0x0u) || (encryptingMethod != 0x0u))
    {
        // 압축, 암호화 없음만 지원함
        errorResult = NRC_RequestOutRange;
    }
    else if((lengthOfDataSize != 0x4u) || (lengthOfAddress != 0x4u))
    {
        // 주소 길이랑 데이터 길이도 4바이트인 경우만 지원
        errorResult = NRC_RequestOutRange;
    }
    else
    {
        stBswDcmData.downloadDataLength = 0u;  // 다운로드 누적 길이
        // 4바이트 합쳐서 다운로드 주소 만듦
        memoryAddress = ((uint32)stBswDcmData.diagDataBufferRx[3] << 24u) | ((uint32)stBswDcmData.diagDataBufferRx[4] << 16u) |
                ((uint32)stBswDcmData.diagDataBufferRx[5] << 8u) | ((uint32)stBswDcmData.diagDataBufferRx[6] << 0u);
        // 다운로드할 총 크기 만듦
        memorySize = ((uint32)stBswDcmData.diagDataBufferRx[7] << 24u) | ((uint32)stBswDcmData.diagDataBufferRx[8] << 16u) |
                ((uint32)stBswDcmData.diagDataBufferRx[9] << 8u) | ((uint32)stBswDcmData.diagDataBufferRx[10] << 0u);


        if((memoryAddress == MEMORY_ADDRESS_APPLICATION) && (memorySize == MEMORY_SIZE_APPLICATION))
        {
            stBswDcmData.downloadMemoryAddress = memoryAddress;
            stBswDcmData.downloadMemorySize = memorySize;
            stBswDcmData.downloadDataLength = 0u;
            stBswDcmData.downloadBlockSequence = 1u;
            errorResult = FALSE;
        }
        else
        {
            // 에러
            stBswDcmData.downloadMemoryAddress = 0u;
            stBswDcmData.downloadMemorySize = 0u;
            stBswDcmData.downloadDataLength = 0u;
            errorResult = NRC_RequestOutRange;
        }

        if(errorResult == FALSE)
        {
            // 키 sha 분산 -> transfer data 전에 해도 됨
            SHA256_Init(&stBswDcmData.signInfo);
            SHA256_Process(&stBswDcmData.signInfo, stBswDcmData.signKey, KEY_SIZE_SIGNATURE);

            stBswDcmData.diagDataLengthTx = 4u;
            stBswDcmData.diagDataBufferTx[0] = (FRAME_TYPE_SINGLE << 4u) | stBswDcmData.diagDataLengthTx;
            stBswDcmData.diagDataBufferTx[1] = SID_RequestDownload + 0x40u;
            stBswDcmData.diagDataBufferTx[2] = 0x20u;   /* maxNumberofBlockLength is 2Byte */
            stBswDcmData.diagDataBufferTx[3] = (MAX_NUMBER_OF_BLOCK_LENGTH & 0xFF00) >> 8u ;
            stBswDcmData.diagDataBufferTx[4] = (MAX_NUMBER_OF_BLOCK_LENGTH & 0x00FF);
        }
    }
    return errorResult;
}

static uint8 transferData(void)
{
    uint8 errorResult;
    uint8 blockSequence = stBswDcmData.diagDataBufferRx[1];  // 블럭 시퀀스 1바이트
    uint16 dataLength = stBswDcmData.diagDataLengthRx-2u;  // 순수 데이터 길이 = 받은 길이 - sid(1) - blocksize(1)
    uint16 decryptLength;

    if(stBswDcmData.diagDataLengthRx < 3u)
    {
        // 데이터 길이 3보다 작으면 오류
        errorResult = NRC_IncorrectMessageLengthOrInvalidFormat;
    }
    else if(blockSequence != stBswDcmData.downloadBlockSequence)
    {
        // 블록 시퀀스가 예상과 다르면 오류
        errorResult = NRC_WrongBlockSequeceNumber;
    }
    else if((stBswDcmData.downloadDataLength + dataLength) > stBswDcmData.downloadMemorySize)
    {
        // 약속한 크기보다 더 보냈을 경우 오류
        errorResult = NRC_TransferDataSuspended;
    }
    else
    {
        decryptLength = dataLength;  // 쓸 데이터 길이
        // 현재 누적 데이터 개수 + 쓸 데이터 길이 > sign을 빼고 써야할 데이터를 넘으면 -> sign이 있는 블록
        if ((stBswDcmData.downloadDataLength + dataLength) > (stBswDcmData.downloadMemorySize - MEMORY_SIZE_SIGNATURE))
        {
            if (stBswDcmData.downloadDataLength >= stBswDcmData.downloadMemorySize - MEMORY_SIZE_SIGNATURE)
            {   // sign 데이터는 바꾸지 않음
                decryptLength = 0;
            }
            else
            {
                // sign이 아닌 부분 개수 계산
                decryptLength = stBswDcmData.downloadMemorySize - MEMORY_SIZE_SIGNATURE - stBswDcmData.downloadDataLength;
            }
        }

        // 복호화 해야 할 만큼 하기
        for (uint16 i = 0; i < decryptLength; i++)
        {
            // RX[2]가 데이터 시작되는 부분
            stBswDcmData.diagDataBufferRx[2 + i] -= 0x05;
        }

        // 정상: 다운로드 할 주소 = 시작 주소 + 누적으로 쓴 데이터 길이 + Secondary에 써야하기 때문에 offset 더하기 , data 주소에 적음
        Rte_Call_BswDcm_rEcuAbsFls_writePflash(stBswDcmData.downloadMemoryAddress +
                    stBswDcmData.downloadDataLength + MEMORY_SECONDARY_OFFSET, dataLength, &stBswDcmData.diagDataBufferRx[2], &errorResult);
        if(errorResult == FALSE)
        {
            // 모든 길이 해시
            SHA256_Process(&stBswDcmData.signInfo, &stBswDcmData.diagDataBufferRx[2], decryptLength);
            // 에러 없으면 정상 응답 메세지
            stBswDcmData.downloadDataLength += dataLength;
            stBswDcmData.downloadBlockSequence++;
            stBswDcmData.diagDataLengthTx = 2u;
            stBswDcmData.diagDataBufferTx[0] = (FRAME_TYPE_SINGLE << 4u) | stBswDcmData.diagDataLengthTx;
            stBswDcmData.diagDataBufferTx[1] = SID_TransferData + 0x40u;
            stBswDcmData.diagDataBufferTx[2] = blockSequence;
        }
        else
        {
            // 에러
            errorResult = NRC_GeneralProgrammingFailure;
        }
    }
    return errorResult;
}

static uint8 requestTransferExit(void)
{
    uint8 errorResult;
    if (stBswDcmData.diagDataLengthRx != 1u)
    {
        // 1 아니면 길이 오류
        errorResult = NRC_IncorrectMessageLengthOrInvalidFormat;
    }
    else if(stBswDcmData.downloadDataLength != stBswDcmData.downloadMemorySize)
    {
        // 다운로드한 데이터 길이와 설정했던 다운로드하고자 한 사이즈가 다르면 오류
        errorResult = NRC_GeneralProgrammingFailure;
    }
    else
    {
        // 정상 : secondary offset 더하고 패턴 작성
        Rte_Call_BswDcm_rEcuAbsFls_writePflash(MEMORY_PATTERN_VALID_APP_ADDR +
                    MEMORY_SECONDARY_OFFSET, MEMORY_DATA_VALID_PATTERN_LENGTH, (P_uint8) MEMORY_DATA_VALID_PATTERN_VALUE, &errorResult);
        if(errorResult == FALSE)
        {
            // trasfer data 끝났으니 종료
            SHA256_Close(&stBswDcmData.signInfo, stBswDcmData.signResult);
            // 정상이면 정상응답
            stBswDcmData.diagDataLengthTx = 1u;
            stBswDcmData.diagDataBufferTx[0] = (FRAME_TYPE_SINGLE << 4u) | stBswDcmData.diagDataLengthTx;
            stBswDcmData.diagDataBufferTx[1] = SID_RequestTransferExit + 0x40u;
        }
        else
        {
            // 에러
            errorResult = NRC_GeneralProgrammingFailure;
        }
    }
    return errorResult;
}
