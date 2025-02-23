/*
 *  Copyright (c) Texas Instruments Incorporated 2023
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

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */
#include <tsn_combase/combase.h>
#include <nrt_flow/dataflow.h>

#define ETH_P_MMRP		0x88F6	/* 802.1Q MMRP */
#define ETH_P_MVRP		0x88F5	/* 802.1Q MVRP */
#define ETH_P_MSRP		0x22EA	/* 802.1Q MSRP */

void rxDefaultDataCb(void *data, int size, int port, void *cbArg)
{
    if (size > 14) {
        uint16_t ethtype = ntohs(*(uint16_t *)((uint8_t *)data + 12));
        EnetAppUtils_print("%s:pkt ethtype=0x%x, size=%d, port=%d\r\n",
                           __func__, ethtype, size, port);
    } else {
        EnetAppUtils_print("%s:pkt size=%d, port=%d\r\n",
                           __func__, size, port);
    }
}

int EnetApp_lldCfgUpdateCb(cb_socket_lldcfg_update_t *update_cfg)
{
    if (update_cfg->proto == ETH_P_1588)
    {
        update_cfg->numRxChannels = 1;
        update_cfg->dmaTxChId = ENET_DMA_TX_CH_PTP;
        update_cfg->dmaRxChId[0] = ENET_DMA_RX_CH_PTP;
        update_cfg->nTxPkts = ENET_DMA_TX_CH_PTP_NUM_PKTS;
        update_cfg->nRxPkts[0] = ENET_DMA_RX_CH_PTP_NUM_PKTS;
        update_cfg->pktSize = ENET_MEM_LARGE_POOL_PKT_SIZE;

    }
    else if (update_cfg->proto == ETH_P_MMRP)
    {
        // Do nothing. We do not support MMRP
    }
    else if (update_cfg->proto == ETH_P_MVRP)
    {
        update_cfg->numRxChannels = 1;
        update_cfg->dmaTxChId = ENET_DMA_TX_CH_MVRP;
        update_cfg->dmaRxChId[0] = ENET_DMA_RX_CH_MVRP;
        update_cfg->nTxPkts = ENET_DMA_TX_CH_MVRP_NUM_PKTS;
        update_cfg->nRxPkts[0] = ENET_DMA_RX_CH_MVRP_NUM_PKTS;
        update_cfg->pktSize = ENET_MEM_LARGE_POOL_PKT_SIZE;
    }
    else if (update_cfg->proto == ETH_P_MSRP)
    {
        update_cfg->numRxChannels = 1;
        update_cfg->dmaTxChId = ENET_DMA_TX_CH_MSRP;
        update_cfg->dmaRxChId[0] = ENET_DMA_RX_CH_MSRP;
        update_cfg->nTxPkts = ENET_DMA_TX_CH_MSRP_NUM_PKTS;
        update_cfg->nRxPkts[0] = ENET_DMA_RX_CH_MSRP_NUM_PKTS;
        update_cfg->pktSize = ENET_MEM_LARGE_POOL_PKT_SIZE;
    }
    else if (update_cfg->proto == ETH_P_TSN)
    {
        update_cfg->numRxChannels = 1;
        update_cfg->dmaTxChId = ENET_DMA_TX_CH_AVTP;
        update_cfg->dmaRxChId[0] = ENET_DMA_RX_CH_AVTP;
        update_cfg->nTxPkts = ENET_DMA_TX_CH_AVTP_NUM_PKTS;
        update_cfg->nRxPkts[0] = ENET_DMA_RX_CH_AVTP_NUM_PKTS;
        update_cfg->pktSize = ENET_MEM_LARGE_POOL_PKT_SIZE;
    }
    return 0;
}
