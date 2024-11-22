/*
 *  Copyright (c) Texas Instruments Incorporated 2020-23
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * \file  ether_ring.c
 *
 * \brief This file contains the implementation of the Ether-ring Driver.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "ether_ring.h"
#include "enet_utils.h"
#include "enet_ethutils.h"
#include <core/enet_dma.h>
#include "kernel/dpl/SystemP.h"
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define CB_HEADER_SIZE                                              4
#define VLAN_HEADER_SIZE                                            sizeof(EthVlanFrameHeader)
#define PACKET_HDR_PLUS_CBLIKE_HDR_LENGTH                           (VLAN_HEADER_SIZE + CB_HEADER_SIZE)
#define ETHERRING_MEMBLOCKS_COUNT                                   128
#define ETHERRING_MEMPOOL_SIZE                                      (ETHERRING_MEMBLOCKS_COUNT * PACKET_HDR_PLUS_CBLIKE_HDR_LENGTH)
#define LOOKUP_TABLE_SIZE                                           256*256
#define MAX_CLASSA_STREAMS                                          3
#define MAX_CLASSD_STREAMS                                          3
#define MAX_SEQUENCE_NUMBER                                         255
#define LOOKUP_CLEAR_TASK_PRIORITY                                  1U
#define LOOKUP_CLEAR_TASK_POOL_PERIOD_USEC                          8000
/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
uint8_t gEtherRing_MemPool[ETHERRING_MEMPOOL_SIZE];

typedef struct
{
	EnetQ etherRingFreeQueue;
    bool etherRingIsMemPoolInitialised;
} EtherRingPool;

typedef struct
{
	int8_t etherRingSeqLookUp[LOOKUP_TABLE_SIZE];
	uint64_t etherRingNonDuplicatedPktCount;
	uint64_t etherRingDuplicatedRxPacketCount;
	uint64_t etherRingSubmittedPacketCount;
	uint64_t etherRingClassRxCount[MAX_CLASSA_STREAMS + MAX_CLASSD_STREAMS];
}EtherRingStats;


/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
EtherRing_Obj EtherRing_ObjectList[MAX_ETHERRING_INSTANCES] =
{
    [0] =
    {
        .isAllocated = false,
        .prevSequenceNumber = 0,
    },
};

static EtherRingPool gEtherRingPool = {
        .etherRingIsMemPoolInitialised = false,
};

static EtherRingStats gEtherRingStats =
{
        .etherRingNonDuplicatedPktCount = 0,
        .etherRingDuplicatedRxPacketCount = 0,
        .etherRingSubmittedPacketCount = 0,
};

EtherRingRxTs_obj gEtherRingRxTs =
{
        .etherRingRxClassATsIndex = 0,
        .etherRingRxClassDTsIndex = 0,
};

static EtherRing_Cfg *gEtherRingCfg;

static EtherRing_ClearLookupPollTaskInfo gEtherRingClearLookupTaskInfo;
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void EtherRing_initMemPool (EtherRingPool *memPool)
{
    if (memPool->etherRingIsMemPoolInitialised == false)
    {
        EnetQueue_initQ(&gEtherRingPool.etherRingFreeQueue);

        uint32_t memPoolIndex = 0;

        for (memPoolIndex = 0; memPoolIndex < ETHERRING_MEMBLOCKS_COUNT; memPoolIndex++)
        {
            uint8_t* pMemBlock =  &gEtherRing_MemPool[memPoolIndex*PACKET_HDR_PLUS_CBLIKE_HDR_LENGTH];
            EnetQueue_enq(&gEtherRingPool.etherRingFreeQueue, (EnetQ_Node*)pMemBlock);
        }

        Enet_assert(EnetQueue_getQCount(&gEtherRingPool.etherRingFreeQueue) == ETHERRING_MEMBLOCKS_COUNT);
        gEtherRingPool.etherRingIsMemPoolInitialised = true;
    }
}


EtherRing_Handle EtherRing_open(Enet_Handle hEnet,
                                uint32_t appCoreId,
                                const void *pEtherRingCfg)
{
    EtherRing_initMemPool(&gEtherRingPool);

    int32_t etherRingIndex;
    EtherRing_Handle hEtherRing = NULL;

    Enet_assert(hEnet != NULL);
    Enet_assert(pEtherRingCfg != NULL);

    gEtherRingCfg = (EtherRing_Cfg*) pEtherRingCfg;
    gEtherRingCfg->isCfg = true;

    for (etherRingIndex = 0U; etherRingIndex < MAX_ETHERRING_INSTANCES;
            etherRingIndex++)
    {
        if (EtherRing_ObjectList[etherRingIndex].isAllocated == false)
        {
            hEtherRing = &EtherRing_ObjectList[etherRingIndex];
            hEtherRing->hEnet = hEnet;
            EtherRing_ObjectList[etherRingIndex].isAllocated = true;
            break;
        }
    }

    return hEtherRing;
}

int32_t EtherRing_close(void *hEtherRing)
{
    int32_t retVal = ENET_SOK;

    Enet_assert(hEtherRing != NULL);

    EtherRing_Handle pRingHandle = (EtherRing_Handle) hEtherRing;

    if(pRingHandle->isAllocated == true)
    {
        pRingHandle->isAllocated = false;
        pRingHandle->hEnet = NULL;
        pRingHandle->hTxCh = NULL;
        pRingHandle->hRxCh = NULL;
    }

    return retVal;
}

void EtherRing_TxDmaHdle_Attach(void *hEtherRing,
                                EnetDma_TxChHandle hTxCh,
                                int32_t txChNum)
{
    Enet_assert(hEtherRing != NULL);

    EtherRing_Handle pRingHandle = (EtherRing_Handle) hEtherRing;
    pRingHandle->hTxCh = hTxCh;

    Enet_assert(txChNum < 8);
    pRingHandle->txChNum = txChNum;
}

void EtherRing_RxDmaHdle_Attach(void *hEtherRing,
                                EnetDma_RxChHandle hRxCh,
                                int32_t rxChNum)
{
    Enet_assert(hEtherRing != NULL);

    EtherRing_Handle pRingHandle = (EtherRing_Handle) hEtherRing;
    pRingHandle->hRxCh = hRxCh;

    Enet_assert(rxChNum < 8);
    pRingHandle->rxChNum = rxChNum;
}

int32_t EtherRing_submitTxPktQ(void *hEtherRing,
                               EtherRing_pktQ *pSubmitQ)
{
    int32_t retVal = ENET_SOK;
    Enet_assert(hEtherRing != NULL);

    EtherRing_pktQ txSubmitQ;
    EtherRing_Handle pRingHandle = (EtherRing_Handle) hEtherRing;
    EnetDma_Pkt *pktInfo = NULL;

    EnetQueue_initQ(&txSubmitQ);

    pktInfo = (EnetDma_Pkt*) EnetQueue_deq(pSubmitQ);

    while (pktInfo != NULL)
    {
        pRingHandle->prevSequenceNumber++;
        EnetApp_addCBLikeHeader(pktInfo, pRingHandle->prevSequenceNumber);
        if (pRingHandle->prevSequenceNumber >= MAX_SEQUENCE_NUMBER)
        {
            pRingHandle->prevSequenceNumber = 0;
        }
        EnetQueue_enq(&txSubmitQ, &pktInfo->node);
        pktInfo = (EnetDma_Pkt*) EnetQueue_deq(pSubmitQ);
    }
    retVal = EnetDma_submitTxPktQ(pRingHandle->hTxCh,
                                  &txSubmitQ);

    return retVal;
}

int32_t EtherRing_retrieveTxPktQ(void *hEtherRing,
                                 EtherRing_pktQ *pRetrieveQ)
{
    int32_t retVal = ENET_SOK;
    Enet_assert(hEtherRing != NULL);

    EtherRing_Handle pRingHandle = (EtherRing_Handle) hEtherRing;
    EnetDma_Pkt *pktInfo = NULL;

    EtherRing_pktQ retrieveQ;
    EnetQueue_initQ(&retrieveQ);


    retVal = EnetDma_retrieveTxPktQ(pRingHandle->hTxCh,
                                    &retrieveQ);

    while(EnetQueue_getQCount(&retrieveQ))
    {
        pktInfo = (EnetDma_Pkt*) EnetQueue_deq(&retrieveQ);

        EnetApp_removeCBLikeHeader(pktInfo);
        EnetQueue_enq(pRetrieveQ, &pktInfo->node);
    }

    return retVal;
}

int32_t EtherRing_submitRxPktQ(void *hEtherRing,
                               EtherRing_pktQ *pSubmitQ)
{
    int32_t retVal = ENET_SOK;
    Enet_assert(hEtherRing != NULL);

    EtherRing_Handle pRingHandle = (EtherRing_Handle) hEtherRing;
    EtherRing_pktQ rxSubmitQ;
    EnetDma_Pkt *pktInfo = NULL;

    EnetQueue_initQ(&rxSubmitQ);

    pktInfo = (EnetDma_Pkt*) EnetQueue_deq(pSubmitQ);

    while (pktInfo != NULL)
    {
        pktInfo->sgList.list[0].bufPtr -= CB_HEADER_SIZE;
        EnetQueue_enq(&rxSubmitQ, &pktInfo->node);
        pktInfo = (EnetDma_Pkt*) EnetQueue_deq(pSubmitQ);
    }
    retVal = EnetDma_submitRxPktQ(pRingHandle->hRxCh, &rxSubmitQ);

    return retVal;
}

int32_t EtherRing_retrieveRxPktQ(void *hEtherRing,
                                 EtherRing_pktQ *pRetrieveQ)
{
    int32_t retVal = ENET_SOK;
    Enet_assert(hEtherRing != NULL);

    EtherRing_Handle pRingHandle = (EtherRing_Handle) hEtherRing;
    EnetDma_Pkt *pktInfo = NULL;
    uint8_t vlanHeaderSize = sizeof(EthVlanFrameHeader);
    uint8_t lastByteMac;
    uint8_t seqNumber;
    uint8_t streamId;
    uint16_t lookupIndex;
    uint8_t* currTimeStampPtr;
    uint64_t currTimeStampValue;

    EtherRing_pktQ rxRetrieveQ;
    EtherRing_pktQ rxDupPktQ;
    EnetQueue_initQ(&rxRetrieveQ);
    EnetQueue_initQ(&rxDupPktQ);


    retVal = EnetDma_retrieveRxPktQ(pRingHandle->hRxCh, &rxRetrieveQ);


    pktInfo = (EnetDma_Pkt*) EnetQueue_deq(&rxRetrieveQ);
    while (pktInfo != NULL)
    {
        /* look-up process for CB packets */
        if (pktInfo->sgList.list[0].bufPtr[vlanHeaderSize + 1] == 0xC1
                & pktInfo->sgList.list[0].bufPtr[vlanHeaderSize] == 0xF1)
        {
            lastByteMac = pktInfo->sgList.list[0].bufPtr[20];
            seqNumber = pktInfo->sgList.list[0].bufPtr[21];

            lookupIndex = (uint16_t) (((uint16_t) lastByteMac << 8) | seqNumber);
            if (gEtherRingStats.etherRingSeqLookUp[lookupIndex] == 0)
            {
                if (pktInfo->tsInfo.rxPktTs)
                {
                    /* capturing the rx timestamps for retrieved CB packets */
                    streamId = pktInfo->sgList.list[0].bufPtr[30];

                    gEtherRingStats.etherRingClassRxCount[streamId]++;
                    if ((gEtherRingRxTs.etherRingRxClassATsIndex
                            < MAX_RX_TIMESTAMPS_STORED) && (streamId == 0))
                    {
                        /* Storing the rxTs for current packet*/
                        gEtherRingRxTs.etherRingTimeStampsRx[0][gEtherRingRxTs.etherRingRxClassATsIndex] =
                                pktInfo->tsInfo.rxPktTs;

                        currTimeStampPtr = pktInfo->sgList.list[0].bufPtr + PACKET_HDR_PLUS_CBLIKE_HDR_LENGTH;
                        currTimeStampValue = *(uint64_t*)currTimeStampPtr;

                        /* Storing the current timestamp received with CB packet*/
                        gEtherRingRxTs.etherRingCurrentTimeStamps[0][gEtherRingRxTs.etherRingRxClassATsIndex] = currTimeStampValue;

                        gEtherRingRxTs.etherRingRxClassATsIndex++;
                    }
                    else if ((gEtherRingRxTs.etherRingRxClassDTsIndex
                            < MAX_RX_TIMESTAMPS_STORED) && (streamId == 3))
                    {
                        /* Storing the rxTs for current packet*/
                        gEtherRingRxTs.etherRingTimeStampsRx[1][gEtherRingRxTs.etherRingRxClassDTsIndex] =
                                pktInfo->tsInfo.rxPktTs;

                        currTimeStampPtr = pktInfo->sgList.list[0].bufPtr + PACKET_HDR_PLUS_CBLIKE_HDR_LENGTH;
                        currTimeStampValue = *(uint64_t*)currTimeStampPtr;
                        /* Storing the current timestamp received with CB packet*/
                        gEtherRingRxTs.etherRingCurrentTimeStamps[1][gEtherRingRxTs.etherRingRxClassDTsIndex] = currTimeStampValue;

                        gEtherRingRxTs.etherRingRxClassDTsIndex++;
                    }
                }

                /* remove the CB header before giving to application */
                memmove(pktInfo->sgList.list[0].bufPtr + CB_HEADER_SIZE,
                        pktInfo->sgList.list[0].bufPtr, VLAN_HEADER_SIZE);
                pktInfo->sgList.list[0].bufPtr += CB_HEADER_SIZE;

                gEtherRingStats.etherRingSeqLookUp[lookupIndex]++;
                gEtherRingStats.etherRingNonDuplicatedPktCount++;
                gEtherRingStats.etherRingDuplicatedRxPacketCount++;
                EnetQueue_enq(pRetrieveQ, &pktInfo->node);
            }
            else if (gEtherRingStats.etherRingSeqLookUp[lookupIndex] == 1)
            {
                gEtherRingStats.etherRingSeqLookUp[lookupIndex] = 0;
                gEtherRingStats.etherRingDuplicatedRxPacketCount++;
                EnetQueue_enq(&rxDupPktQ, &pktInfo->node);
                EtherRing_submitRxPktQ(pRingHandle, &rxDupPktQ);
            }
            else
            {
                EnetQueue_enq(pRetrieveQ, &pktInfo->node);
            }
        }
        else
        {
            EnetQueue_enq(&rxDupPktQ, &pktInfo->node);
            EtherRing_submitRxPktQ(pRingHandle, &rxDupPktQ);
        }
        pktInfo = (EnetDma_Pkt*) EnetQueue_deq(&rxRetrieveQ);
    }

    return retVal;
}

void EnetApp_addCBLikeHeader(EnetDma_Pkt *pktInfo,
                             uint16_t seqNumber)
{
    uint8_t *headerWithCB = NULL;

    if (EnetQueue_getQCount(&gEtherRingPool.etherRingFreeQueue) > 0)
    {
        headerWithCB = (uint8_t*)EnetQueue_deq(&gEtherRingPool.etherRingFreeQueue);
    }

    Enet_assert(headerWithCB != NULL);
    pktInfo->sgList.list[1] = pktInfo->sgList.list[0];
    pktInfo->sgList.list[1].bufPtr += VLAN_HEADER_SIZE;
    pktInfo->sgList.list[1].segmentFilledLen -= VLAN_HEADER_SIZE;
    pktInfo->sgList.list[0].segmentFilledLen = PACKET_HDR_PLUS_CBLIKE_HDR_LENGTH;

    memcpy(headerWithCB, pktInfo->sgList.list[0].bufPtr, VLAN_HEADER_SIZE);

    /* F1-C1 EtherType for CB */
    headerWithCB[VLAN_HEADER_SIZE + 1] = 0xC1;
    headerWithCB[VLAN_HEADER_SIZE] = 0xF1;

    /* Adding seq_number to the Header */
    headerWithCB[VLAN_HEADER_SIZE + 3] = seqNumber & 0xFF;

    /* Last byte of host macAddr */
    headerWithCB[VLAN_HEADER_SIZE + 2] = gEtherRingCfg->hostMacAddLastByte;

    pktInfo->sgList.list[0].bufPtr = headerWithCB;

    pktInfo->sgList.numScatterSegments = 2;
}

void EnetApp_removeCBLikeHeader(EnetDma_Pkt *pktInfo)
{
    EnetQueue_enq(&gEtherRingPool.etherRingFreeQueue, (EnetQ_Node*)pktInfo->sgList.list[0].bufPtr);
    pktInfo->sgList.list[0].bufPtr = pktInfo->sgList.list[1].bufPtr - VLAN_HEADER_SIZE;
    pktInfo->sgList.numScatterSegments = 1;
}

static void EtherRing_clearLookupPollTask(void *args)
{
    while (!gEtherRingClearLookupTaskInfo.shutDownFlag)
    {
        SemaphoreP_pend(&gEtherRingClearLookupTaskInfo.sem, SystemP_WAIT_FOREVER);
        memset(gEtherRingStats.etherRingSeqLookUp, 0, LOOKUP_TABLE_SIZE);
    }
    SemaphoreP_post(&gEtherRingClearLookupTaskInfo.shutDownSemObj);
}

static void EnetApp_postclearLookupPollLink(ClockP_Object *clkObj, void *arg)
{
    if (arg != NULL)
    {
        SemaphoreP_Object *hPollSem = (SemaphoreP_Object *) arg;
        SemaphoreP_post(hPollSem);
    }
}

int32_t EtherRing_createClearLookupPollTask()
{
    TaskP_Params params;
    int32_t status;
    ClockP_Params clkPrms;

    /*Initialize semaphore to call synchronize the poll function with a timer*/
    status = SemaphoreP_constructBinary(&gEtherRingClearLookupTaskInfo.sem, 0U);
    Enet_assert(status == SystemP_SUCCESS);

    /*Initialize semaphore to call synchronize the poll function with a timer*/
    status = SemaphoreP_constructBinary(&gEtherRingClearLookupTaskInfo.shutDownSemObj, 0U);
    Enet_assert(status == SystemP_SUCCESS);

    /* Initialize the poll function as a thread */
    TaskP_Params_init(&params);
    params.name           = "Clear Lookup Table Task";
    params.priority       = LOOKUP_CLEAR_TASK_PRIORITY;
    params.stack          = gEtherRingClearLookupTaskInfo.gEnetTaskStackPolling;
    params.stackSize      = sizeof(gEtherRingClearLookupTaskInfo.gEnetTaskStackPolling);
    params.args           = (void*)&gEtherRingStats;
    params.taskMain       = &EtherRing_clearLookupPollTask;

    status = TaskP_construct(&gEtherRingClearLookupTaskInfo.task, &params);
    Enet_assert(status == SystemP_SUCCESS);

    ClockP_Params_init(&clkPrms);
    clkPrms.start     = 0;
    clkPrms.period    = ClockP_usecToTicks(LOOKUP_CLEAR_TASK_POOL_PERIOD_USEC);
    clkPrms.args      = &gEtherRingClearLookupTaskInfo.sem;
    clkPrms.callback  = &EnetApp_postclearLookupPollLink;
    clkPrms.timeout   = ClockP_usecToTicks(LOOKUP_CLEAR_TASK_POOL_PERIOD_USEC);

    /* Creating timer and setting timer callback function*/
    status = ClockP_construct(&gEtherRingClearLookupTaskInfo.pollLinkClkObj, &clkPrms);
    if (status == SystemP_SUCCESS)
    {
        /* Set timer expiry time in OS ticks */
        ClockP_setTimeout(&gEtherRingClearLookupTaskInfo.pollLinkClkObj,
                          ClockP_usecToTicks(LOOKUP_CLEAR_TASK_POOL_PERIOD_USEC));
        ClockP_start(&gEtherRingClearLookupTaskInfo.pollLinkClkObj);
    }
    else
    {
        Enet_assert(status == SystemP_SUCCESS);
    }

    return status;
}
