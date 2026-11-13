#include "Rte_BswCom.h"
#include "Definition.h"
#include "Utility.h"

typedef struct {
    uint8 canMsgBuffer[TOTAL_CAN_MSG_COUNT][MAX_CAN_MSG_DLC];
} ST_BswComData;
static ST_BswComData stBswComData;

void REcbBswCom_initialize(void)
{
    Rte_Call_BswCom_rEcuAbsCan_setPduInfo(RX_MSG_IDX_PHY_REQ, RX_MSG_LEN_PHY_REQ, RX_MSG_ID_PHY_REQ, stBswComData.canMsgBuffer[RX_MSG_IDX_PHY_REQ]);
    Rte_Call_BswCom_rEcuAbsCan_setPduInfo(RX_MSG_IDX_FUN_REQ, RX_MSG_LEN_FUN_REQ, RX_MSG_ID_FUN_REQ, stBswComData.canMsgBuffer[RX_MSG_IDX_FUN_REQ]);
    Rte_Call_BswCom_rEcuAbsCan_setPduInfo(RX_MSG_IDX_CCP_REQ, RX_MSG_LEN_CCP_REQ, RX_MSG_ID_CCP_REQ, stBswComData.canMsgBuffer[RX_MSG_IDX_CCP_REQ]);
    Rte_Call_BswCom_rEcuAbsCan_setPduInfo(RX_MSG_IDX_SAS_01, RX_MSG_LEN_SAS_01, RX_MSG_ID_SAS_01, stBswComData.canMsgBuffer[RX_MSG_IDX_SAS_01]);

    Rte_Call_BswCom_rEcuAbsCan_setPduInfo(TX_MSG_IDX_SDC_01, TX_MSG_LEN_SDC_01, TX_MSG_ID_SDC_01, stBswComData.canMsgBuffer[TX_MSG_IDX_SDC_01]);
    Rte_Call_BswCom_rEcuAbsCan_setPduInfo(TX_MSG_IDX_PHY_RES, TX_MSG_LEN_PHY_RES, TX_MSG_ID_PHY_RES, stBswComData.canMsgBuffer[TX_MSG_IDX_PHY_RES]);
    Rte_Call_BswCom_rEcuAbsCan_setPduInfo(TX_MSG_IDX_CCP_RES, TX_MSG_LEN_CCP_RES, TX_MSG_ID_CCP_RES, stBswComData.canMsgBuffer[TX_MSG_IDX_CCP_RES]);
    Rte_Call_BswCom_rEcuAbsCan_setPduInfo(TX_MSG_IDX_CCP_MON0, TX_MSG_LEN_CCP_MON_DEFAULT, TX_MSG_ID_CCP_MON0, stBswComData.canMsgBuffer[TX_MSG_IDX_CCP_MON0]);
    Rte_Call_BswCom_rEcuAbsCan_setPduInfo(TX_MSG_IDX_CCP_MON1, TX_MSG_LEN_CCP_MON_DEFAULT, TX_MSG_ID_CCP_MON1, stBswComData.canMsgBuffer[TX_MSG_IDX_CCP_MON1]);
    Rte_Call_BswCom_rEcuAbsCan_setPduInfo(TX_MSG_IDX_CCP_MON2, TX_MSG_LEN_CCP_MON_DEFAULT, TX_MSG_ID_CCP_MON2, stBswComData.canMsgBuffer[TX_MSG_IDX_CCP_MON2]);
    Rte_Call_BswCom_rEcuAbsCan_setPduInfo(TX_MSG_IDX_CCP_MON3, TX_MSG_LEN_CCP_MON_DEFAULT, TX_MSG_ID_CCP_MON3, stBswComData.canMsgBuffer[TX_MSG_IDX_CCP_MON3]);
    Rte_Call_BswCom_rEcuAbsCan_setPduInfo(TX_MSG_IDX_CCP_MON4, TX_MSG_LEN_CCP_MON_DEFAULT, TX_MSG_ID_CCP_MON4, stBswComData.canMsgBuffer[TX_MSG_IDX_CCP_MON4]);
    Rte_Call_BswCom_rEcuAbsCan_setPduInfo(TX_MSG_IDX_CCP_MON5, TX_MSG_LEN_CCP_MON_DEFAULT, TX_MSG_ID_CCP_MON5, stBswComData.canMsgBuffer[TX_MSG_IDX_CCP_MON5]);
    Rte_Call_BswCom_rEcuAbsCan_setPduInfo(TX_MSG_IDX_CCP_MON6, TX_MSG_LEN_CCP_MON_DEFAULT, TX_MSG_ID_CCP_MON6, stBswComData.canMsgBuffer[TX_MSG_IDX_CCP_MON6]);
    Rte_Call_BswCom_rEcuAbsCan_setPduInfo(TX_MSG_IDX_CCP_MON7, TX_MSG_LEN_CCP_MON_DEFAULT, TX_MSG_ID_CCP_MON7, stBswComData.canMsgBuffer[TX_MSG_IDX_CCP_MON7]);
}

void REtBswCom_bswComMain(void)
{
    (void) 0;
}

uint8 handle;
uint8 length;
uint32 id;
uint8 data[MAX_CAN_MSG_DLC];

void REcbBswCom_canRxIndication(void)
{
    UT_getCanRxIndicationData(&handle, &length, &id, data);
}
