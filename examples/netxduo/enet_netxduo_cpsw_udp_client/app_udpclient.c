/*
 *  Copyright (C) 2018-2024 Texas Instruments Incorporated
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
/*                             Include Files                                  */
/* ========================================================================== */

/* C runtime includes */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* NetX includes */
#include <netxduo_enet.h>
#include <tx_port.h>
#include <nx_api.h>
#include <nxd_dhcp_client.h>

/* App includes */
#include "app_cpswconfighandler.h"

/* SDK includes */
#include "ti_board_config.h"
#include "ti_enet_netxduo.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_board_config.h"
#include "ti_enet_open_close.h"
#include "ti_enet_config.h"
#include <enet_apputils.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>


#if (NETXDUO_COUNT > 1u)
#error "This example does not support more than one Netx instance."
#endif


/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define PACKET_SIZE  1536
#define POOL_SIZE    ((sizeof(NX_PACKET) + PACKET_SIZE) * (ENET_SYSCFG_TOTAL_NUM_RX_PKT + ENET_SYSCFG_TOTAL_NUM_TX_PKT))

#define IP_THREAD_STACK_SIZE           (8192u)
#define IP_ARP_THREAD_STACK_SIZE       (8192u)

#define APP_NUM_ITERATIONS             (2u)
#define APP_SEND_DATA_NUM_ITERATIONS   (5u)

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static const uint8_t BROADCAST_MAC_ADDRESS[ENET_MAC_ADDR_LEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

static uint8_t gIpThreadStack[IP_THREAD_STACK_SIZE]__attribute__((aligned(ENET_UTILS_CACHELINE_SIZE)));
static uint8_t gIpArpThreadStack[IP_ARP_THREAD_STACK_SIZE]__attribute__((aligned(ENET_UTILS_CACHELINE_SIZE)));
static uint8_t gPoolMem[POOL_SIZE]__attribute__((aligned(ENET_UTILS_CACHELINE_SIZE)));

static char gTransmitBuf[PACKET_SIZE];
static NX_PACKET_POOL gPacketPool;
static NX_IP gIp;
static NX_DHCP gDhcpClient;
static NX_UDP_SOCKET gClientSocket;


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int netxduo_cpsw_main(ULONG arg)
{
    Enet_Type enetType;
    uint32_t instId;
    Enet_Handle hEnet;
    NX_PACKET *pPacket;
    ULONG packetLength;
    ULONG bufLength;
    NXD_ADDRESS server_address;
    Enet_MacPort macPort;
    ULONG ipAddr;
    ULONG netMask;
    uint32_t rxChCnt;
    uint32_t txChCnt;
    const char *pIfName;
    size_t ifIx;
    size_t dfltIfIx;
    ULONG actual_status;
    nx_enet_drv_rx_ch_hndl_t ifRxChs[ENET_NETX_MAX_RX_CHANNELS_PER_PHERIPHERAL];
    nx_enet_drv_tx_ch_hndl_t ifTxChs[ENET_NETX_MAX_RX_CHANNELS_PER_PHERIPHERAL];
    uint32_t chIds[ENET_NETX_MAX_RX_CHANNELS_PER_PHERIPHERAL];
    nx_enet_drv_rx_ch_hndl_t rxChs[ENET_SYSCFG_RX_FLOWS_NUM];
    nx_enet_drv_tx_ch_hndl_t txChs[ENET_SYSCFG_TX_CHANNELS_NUM];
    EnetApp_GetMacAddrOutArgs outArgs;
    bool isLinked;
    int32_t status = ENET_SOK;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("=============================\r\n");
    DebugP_log("   CPSW NETXDUO UDP CLIENT   \r\n");
    DebugP_log("=============================\r\n");

    EnetApp_driverInit();

    EnetApp_getEnetInstInfo(CONFIG_ENET_CPSW0, &enetType, &instId);

    EnetAppUtils_enableClocks(enetType, instId);
    EnetApp_driverInit();

    status = EnetApp_driverOpen(enetType, instId);
    DebugP_assert(status == ENET_SOK);


    EnetApp_addMCastEntry(enetType, instId, EnetSoc_getCoreId(), BROADCAST_MAC_ADDRESS, CPSW_ALE_ALL_PORTS_MASK);


    /* Allocate NetX Rx channel and corresponding buffers. */
    for(size_t k = 0u; k < ENET_SYSCFG_RX_FLOWS_NUM; k++) {

        EnetApp_GetDmaHandleInArgs inArgs = {0};
        EnetApp_GetRxDmaHandleOutArgs outArgs;

        EnetApp_getRxDmaHandle(k, &inArgs, &outArgs);

        EnetAppUtils_assert(outArgs.hRxCh != NULL);
        NetxEnetDriver_allocRxCh(outArgs.hRxCh, outArgs.maxNumRxPkts, &rxChs[k]);
    }

    /* Allocate NetX Tx channel and corresponding buffers. */
    for (size_t k = 0u; k < ENET_SYSCFG_TX_CHANNELS_NUM; k++) {

        EnetApp_GetDmaHandleInArgs inArgs = {0};
        EnetApp_GetTxDmaHandleOutArgs outArgs;

        EnetApp_getTxDmaHandle(k, &inArgs, &outArgs);

        EnetAppUtils_assert(outArgs.hTxCh != NULL);
        NetxEnetDriver_allocTxCh(outArgs.hTxCh, outArgs.maxNumTxPkts, &txChs[k]);
    }

    /* Allocate network interfaces and bind to corresponding DMA channels. */
    dfltIfIx = NetxEnetApp_getDefaultIfIdx();
    for (size_t ifCtr = 0u; ifCtr < NETXDUO_IF_COUNT; ifCtr++) {

        ifIx = ifCtr == 0u ? dfltIfIx : (dfltIfIx == 0) ? 1 : 0;
        pIfName = ifCtr == 0u ? "PRI" : "SEC";

        NetxEnetApp_getRxChIDs(0, 0, &rxChCnt, &chIds[0]);
        for (size_t k = 0u; k < rxChCnt; k++) {
            ifRxChs[k] = rxChs[chIds[k]];
        }

        EnetApp_getMacAddress(chIds[0], &outArgs);

        NetxEnetApp_getTxChIDs(0, 0, &txChCnt, &chIds[0]);
        for (size_t k = 0u; k < txChCnt; k++) {
            ifTxChs[k] = txChs[chIds[k]];
        }

        macPort = NetxEnetApp_getMacPort(0, ifIx);

        EnetAppUtils_assert(ifIx < outArgs.macAddressCnt);
        NetxEnetDriver_allocIf(pIfName, macPort, &outArgs.macAddr[ifIx][0], &ifRxChs[0], rxChCnt, ifTxChs, txChCnt);
    }


    /* Wait for the link on default interface to come up. */
    macPort = NetxEnetApp_getMacPort(0, dfltIfIx);
    NetxEnetApp_getEnetTypeAndIdFromIfIdx(0, dfltIfIx, &enetType, &instId);
    hEnet = Enet_getHandle(enetType, instId);

    isLinked = false;
    while (!isLinked) {

        EnetAppUtils_print("Waiting for link up...\n");
        isLinked = EnetApp_isPortLinked(hEnet);

        tx_thread_sleep(2u * TX_TIMER_TICKS_PER_SECOND);
    }


    /* Initialize the NetX system.  */
    nx_system_initialize();

    /* Create a packet pool.  */
    status = nx_packet_pool_create(&gPacketPool, "NetX Main Packet Pool", PACKET_SIZE, &gPoolMem[0], POOL_SIZE);
    EnetAppUtils_assert(status == NX_SUCCESS);


    /* Create an IP instance.  */
    status = nx_ip_create(&gIp, "NetX IP Instance 0", IP_ADDRESS(0, 0, 0, 0), 0xFFFFFF00UL, &gPacketPool, _nx_enet_driver, (void *)&gIpThreadStack[0], IP_THREAD_STACK_SIZE, 1);
    EnetAppUtils_assert(status == NX_SUCCESS);


    /* Enable ARP */
    status = nx_arp_enable(&gIp, (void *)&gIpArpThreadStack[0], IP_ARP_THREAD_STACK_SIZE);
    EnetAppUtils_assert(status == NX_SUCCESS);

    /* Enable ICMP */
    status = nxd_icmp_enable(&gIp);
    EnetAppUtils_assert(status == NX_SUCCESS);

    /* Enable UDP */
    status = nx_udp_enable(&gIp);
    EnetAppUtils_assert(status == NX_SUCCESS);


    /* Create the DHCP instance.  */
    status = nx_dhcp_create(&gDhcpClient, &gIp, "DHCP-CLIENT");
    EnetAppUtils_assert(status == NX_SUCCESS);

    nx_dhcp_interface_enable(&gDhcpClient, 0u);

    /* Start the DHCP Client.  */
    status = nx_dhcp_interface_start(&gDhcpClient, 0);
    EnetAppUtils_assert(status == NX_SUCCESS);


    /* Wait for DHCP to assign the IP address.  */
    EnetAppUtils_print("Waiting for address from DHCP server on primary interface...\n");
    do {

        /* Check for address resolution.  */
        status = nx_ip_interface_status_check(&gIp, 0u, NX_IP_ADDRESS_RESOLVED, (ULONG *) &actual_status, NX_IP_PERIODIC_RATE);

        tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND);

    } while ((actual_status & NX_IP_ADDRESS_RESOLVED) != NX_IP_ADDRESS_RESOLVED);


    /* Get primary interface address. */
    status = nx_ip_interface_address_get(&gIp, 0u, &ipAddr, &netMask);
    EnetAppUtils_assert(status == NX_SUCCESS);

    DebugP_log("Local Interface IP is: %lu.%lu.%lu.%lu\n", ((ipAddr >> 24u) & 0xFF), ((ipAddr >> 16u) & 0xFF), ((ipAddr >> 8u) & 0xFF), (ipAddr & 0xFF));


    /* Wait for the user to enter the server IP address. */
    {
        ULONG a,b,c,d;
        EnetAppUtils_print("Enter the server IP address.\n");
        DebugP_scanf("%lu.%lu.%lu.%lu", &a, &b, &c, &d);

        /* set the TCP server addresses. */
        server_address.nxd_ip_version = NX_IP_VERSION_V4;
        server_address.nxd_ip_address.v4 = IP_ADDRESS(a, b, c, d);
    }


    /* Loop to repeat things over and over again!  */
    for (uint32_t connIx = 0u; connIx < APP_NUM_ITERATIONS; connIx++)
    {
        /* Create a UDP socket.  */
        status = nx_udp_socket_create(&gIp, &gClientSocket, "Socket 0", NX_IP_NORMAL, NX_FRAGMENT_OKAY, 0x80, 5);
        EnetAppUtils_assert(status == NX_SUCCESS);

        DebugP_log("UDP socket created\r\n");

        /* Bind the socket.  */
        status =  nx_udp_socket_bind(&gClientSocket, NX_ANY_PORT, NX_WAIT_FOREVER);
        EnetAppUtils_assert(status == NX_SUCCESS);

        for (uint32_t packetIx = 0u; packetIx < APP_SEND_DATA_NUM_ITERATIONS; packetIx++)
        {

            /* Allocate a packet.  */
            status =  nx_packet_allocate(&gPacketPool, &pPacket, NX_TCP_PACKET, NX_WAIT_FOREVER);
            EnetAppUtils_assert(status == NX_SUCCESS);

            /* Append data to the packet. */
            memset(&gTransmitBuf, 0, sizeof(gTransmitBuf));
            bufLength = snprintf(gTransmitBuf, sizeof(gTransmitBuf), "Hello over TCP %d", packetIx+1);
            nx_packet_data_append(pPacket, gTransmitBuf, bufLength, &gPacketPool, TX_WAIT_FOREVER);

            status =  nx_packet_length_get(pPacket, &packetLength);
            EnetAppUtils_assert((status == NX_SUCCESS) && (packetLength == bufLength));


            /* Send the packet out on the primary interface. */
            status = nxd_udp_socket_source_send(&gClientSocket, pPacket, &server_address, 8888, 0u);
            if (status == NX_SUCCESS)
            {
                DebugP_log("\"%s\" was sent to the Server\r\n", gTransmitBuf);
            }
            else
            {
                DebugP_log("couldn't send packet to server\r\n");
                continue;
            }

            status = nx_udp_socket_receive(&gClientSocket, &pPacket, NX_WAIT_FOREVER);
            EnetAppUtils_assert(status == NX_SUCCESS);

            nx_packet_data_retrieve(pPacket, (void *)&gTransmitBuf[0], &packetLength);
            if ((status == NX_SUCCESS) && (packetLength == bufLength))
            {
                DebugP_log("Successfully received the packet %d\r\n", packetIx+1);

                status = nx_packet_release(pPacket);
                EnetAppUtils_assert(status == NX_SUCCESS);
            }
            else
            {
                DebugP_log("No response from server\r\n");
            }
        }

        /* Unbind the socket.  */
        status =  nx_udp_socket_unbind(&gClientSocket);
        EnetAppUtils_assert(status == NX_SUCCESS);

        /* Delete the socket.  */
        status =  nx_udp_socket_delete(&gClientSocket);
        EnetAppUtils_assert(status == NX_SUCCESS);

        DebugP_log("Socket deleted\r\n");

        tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND);
    }
    DebugP_log("Done\r\n");

    return 0;
}
