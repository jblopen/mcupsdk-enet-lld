/*
 * Copyright (c) 2001,2002 Florian Schulze.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the authors nor the names of the contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * app_main.c - This file is part of lwIP test
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
/* lwIP core includes */
#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/tcpip.h"
#include "lwip/dhcp.h"

#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/ClockP.h>
#include <enet_apputils.h>
#include <enet_board.h>
#include <lwip/igmp.h>
#include "ti_board_config.h"
#include "ti_board_open_close.h"
#include "ti_drivers_open_close.h"
#include "ti_enet_config.h"
#include "ti_enet_open_close.h"
#include "app_cpswconfighandler.h"
#include "app_udpserver.h"
#include "ti_enet_lwipif.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define IGMP_MCAST_MAC_ADDR          { 0x01, 0x00, 0x5E, 0xFF, 0xFF, 0xFF }
static const uint8_t BROADCAST_MAC_ADDRESS[ENET_MAC_ADDR_LEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
static void App_printCpuLoad();

static void App_tcpipInitCompleteCb(void *pArg);

static void App_setupNetif();

static void App_allocateIPAddress();

static void App_setupNetworkStack();

static void App_shutdownNetworkStack();

static void App_netifStatusChangeCb(struct netif *state_netif);

static void App_netifLinkChangeCb(struct netif *state_netif);

static inline int32_t App_isNetworkUp(struct netif* netif_);

static void App_setIgmpCpswMacFilter(struct netif *netif);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* dhcp struct for the ethernet netif */
static struct dhcp g_netifDhcp[ENET_SYSCFG_NETIF_COUNT];
struct netif *g_pNetif[ENET_SYSCFG_NETIF_COUNT];

/* Handle to the Application interface for the LwIPIf Layer
 */
LwipifEnetApp_Handle hlwipIfApp = NULL;
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int appMain(void *args)
{
    Enet_Type enetType;
    uint32_t instId;

    DebugP_log("============================\r\n");
    DebugP_log("  CPSW LWIP IGMP UDP SERVER \r\n");
    DebugP_log("============================\r\n");

    EnetApp_getEnetInstInfo(CONFIG_ENET_CPSW0, &enetType, &instId);

    EnetAppUtils_enableClocks(enetType, instId);

    EnetApp_driverInit();

    const int32_t status = EnetApp_driverOpen(enetType, instId);
    if (ENET_SOK != status)
    {
        EnetAppUtils_print("Failed to open ENET: %d\r\n", status);
        EnetAppUtils_assert(false);
        return -1;
    }

    EnetApp_addMCastEntry(enetType,
                          instId,
                          EnetSoc_getCoreId(),
                          BROADCAST_MAC_ADDRESS,
                          CPSW_ALE_ALL_PORTS_MASK);

    App_setupNetworkStack();

    while (false == App_isNetworkUp(netif_default))
    {
        DebugP_log("Waiting for network UP ...\r\n");
        ClockP_sleep(2);
    }

    DebugP_log("Network is UP ...\r\n");
    ClockP_sleep(1);
    AppUdp_startServer();

    while (1)
    {
        ClockP_usleep(1000);
        App_printCpuLoad();
    }

    App_shutdownNetworkStack();

    EnetApp_driverDeInit();
    return 0;
}

static void App_setupNetworkStack()
{
    sys_sem_t pInitSem;
    const err_t err = sys_sem_new(&pInitSem, 0);
    EnetAppUtils_assert(err == ERR_OK);

    tcpip_init(App_tcpipInitCompleteCb, &pInitSem);

    /* wait for TCP/IP initialization to complete */
    sys_sem_wait(&pInitSem);
    sys_sem_free(&pInitSem);

    return;
}

static void App_shutdownNetworkStack()
{
    LwipifEnetApp_netifClose(hlwipIfApp, NETIF_INST_ID0);
    return;
}

static void App_tcpipInitCompleteCb(void *pArg)
{
    sys_sem_t *pSem = (sys_sem_t*)pArg;
    EnetAppUtils_assert(pArg != NULL);

    /* init randomizer again (seed per thread) */
    srand((unsigned int)sys_now()/1000);

    App_setupNetif();

    App_allocateIPAddress();

    sys_sem_signal(pSem);
}

static void App_setupNetif()
{
    ip4_addr_t ipaddr, netmask, gw;

    ip4_addr_set_zero(&gw);
    ip4_addr_set_zero(&ipaddr);
    ip4_addr_set_zero(&netmask);

    DebugP_log("Starting lwIP, local interface IP is dhcp-enabled\r\n");
    hlwipIfApp = LwipifEnetApp_getHandle();
    for (uint32_t i = 0U; i < ENET_SYSCFG_NETIF_COUNT; i++)
    {
        /* Open the netif and get it populated*/
        g_pNetif[i] = LwipifEnetApp_netifOpen(hlwipIfApp, NETIF_INST_ID0 + i, &ipaddr, &netmask, &gw);
        App_setIgmpCpswMacFilter(g_pNetif[i]);
        igmp_start(g_pNetif[i]);
        netif_set_status_callback(g_pNetif[i], App_netifStatusChangeCb);
        netif_set_link_callback(g_pNetif[i], App_netifLinkChangeCb);
        netif_set_up(g_pNetif[NETIF_INST_ID0 + i]);
    }
    LwipifEnetApp_startSchedule(hlwipIfApp, g_pNetif[ENET_SYSCFG_DEFAULT_NETIF_IDX]);
}

static void App_allocateIPAddress()
{
    sys_lock_tcpip_core();
    for (uint32_t  i = 0U; i < ENET_SYSCFG_NETIF_COUNT; i++)
    {
        dhcp_set_struct(g_pNetif[NETIF_INST_ID0 + i], &g_netifDhcp[NETIF_INST_ID0 + i]);

        const err_t err = dhcp_start(g_pNetif[NETIF_INST_ID0 + i]);
        EnetAppUtils_assert(err == ERR_OK);
    }
    sys_unlock_tcpip_core();
    return;
}

static void App_netifStatusChangeCb(struct netif *pNetif)
{
    if (netif_is_up(pNetif))
    {
        DebugP_log("Enet IF UP Event. Local interface IP:%s\r\n",
                    ip4addr_ntoa(netif_ip4_addr(pNetif)));
    }
    else
    {
        DebugP_log("Enet IF DOWN Event\r\n");
    }
    return;
}

static void App_netifLinkChangeCb(struct netif *pNetif)
{
    if (netif_is_link_up(pNetif))
    {
        DebugP_log("Network Link UP Event\r\n");
    }
    else
    {
        DebugP_log("Network Link DOWN Event\r\n");
    }
    return;
}

static int32_t App_isNetworkUp(struct netif* netif_)
{
    return (netif_is_up(netif_) && netif_is_link_up(netif_) && !ip4_addr_isany_val(*netif_ip4_addr(netif_)));
}

static void App_printCpuLoad()
{
    static uint32_t startTime_ms = 0;
    const  uint32_t currTime_ms  = ClockP_getTimeUsec()/1000;
    const  uint32_t printInterval_ms = 5000;

    if (startTime_ms == 0)
    {
        startTime_ms = currTime_ms;
    }
    else if ( (currTime_ms - startTime_ms) > printInterval_ms )
    {
        const uint32_t cpuLoad = TaskP_loadGetTotalCpuLoad();

        DebugP_log(" %6d.%3ds : CPU load = %3d.%02d %%\r\n",
                    currTime_ms/1000, currTime_ms%1000,
                    cpuLoad/100, cpuLoad%100 );

        startTime_ms = currTime_ms;
        TaskP_loadResetAll();
    }
    return;
}

static err_t App_igmpMacFilter(struct netif *netif, const ip4_addr_t *group, enum netif_mac_filter_action action)
{
    Enet_Handle hEnet;
    Enet_Type enetType;
    uint32_t instId;
    err_t retVal = ERR_OK;

    /* Convert the Mcast ip address to Mcast MacAddr */
    /* 0x01005e000000 + (ip & 0x7FFFFF) */
    uint8_t mcastMacAddr[ENET_MAC_ADDR_LEN] = IGMP_MCAST_MAC_ADDR;

    mcastMacAddr[3] = (uint8_t) ((ip4_addr2(group)) & 0x7f);
    mcastMacAddr[4] = (uint8_t) ip4_addr3(group);
    mcastMacAddr[5] = (uint8_t) ip4_addr4(group);

    EnetApp_getEnetInstInfo(CONFIG_ENET_CPSW0, &enetType,
                            &instId);
    hEnet = Enet_getHandle(enetType, instId);

    if (action == NETIF_ADD_MAC_FILTER)
    {
        if (EnetAppUtils_addAllPortMcastMembership(hEnet, mcastMacAddr) != ENET_SOK)
        {
            retVal = ERR_IF;
        }
    }
    else if (action == NETIF_DEL_MAC_FILTER)
    {
       if (EnetAppUtils_delAllPortMcastMembership(hEnet, mcastMacAddr) != ENET_SOK)
       {
           retVal = ERR_IF;
       }
    }
    else
    {
        EnetAppUtils_print("EnetAppUtils_igmpMacFilter failed due to invalid argument : %s\n", action);
        retVal = ERR_ARG;
    }
    return retVal;
}

static void App_setIgmpCpswMacFilter(struct netif *netif)
{
    netif->flags |= NETIF_FLAG_IGMP;
    netif_set_igmp_mac_filter(netif, App_igmpMacFilter);
}
