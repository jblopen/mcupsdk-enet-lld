/*
 *  Copyright (c) Texas Instruments Incorporated 2024
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
 *
 */

/*!
 * \file  dp83tg721.c
 *
 * \brief This file contains the implementation of the DP83TG721 PHY.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <include/core/enet_utils.h>
#include <priv/core/enet_trace_priv.h>
#include <include/phy/enetphy.h>
#include <include/phy/dp83tg721.h>

#include "enetphy_priv.h"
#include "generic_phy.h"
#include "dp83tg721_priv.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define DP83TG721_OUI    (0x080028U)
#define DP83TG721_REV    (0U)
#define DP83TG721_MODEL  (0x29U)

#define SEC_NSEC        1000000000UL

#define PTP_VERSION     2
#define PSF_TX          0x1000
#define PSF_RX          0x2000
#define PSF_EVNT        0x4000
#define DP83TG721_ONLY_TIMESTAMP_PTP_EVENTS 0

#define NUM_TRIGGER       2
#define NUM_EVENTS        2
#define START_TRIGGER_IDX 0
#define START_EVENT_IDX   NUM_TRIGGER
#define GPIO_NUM_PINS     (NUM_TRIGGER+NUM_EVENTS)

/* GPIO pins for event capture and trigger */
#define LED_0  1
#define LED_1  2
#define STRP_1 3
#define CLKOUT 4

#define WAIT_TXTS_TIMEOUT    2 /* ticks */

/*
 * The addition process is a pipelined process that takes two clock cycles at 4 ns
 * each at the default clock rate. When adjusting the clock, the value added should
 * include 8 ns to compensate for the 2-cycle addition.
 * For example, to adjust the clock by +100 ns, the actual value added should
 * be 108 ns. To subtract 100 ns, the actual value subtracted should be 92 ns.
 */
#define PTP_CLOCK_FREQ_HZ     250000000 //250Mhz PLL
#define PTP_CLOCK_PERIOD_NSEC (SEC_NSEC/PTP_CLOCK_FREQ_HZ)
#define ADJTIME_FIX           (PTP_CLOCK_PERIOD_NSEC*2)

/*
 * Event timestamp values must be adjusted by x ns (3 times period of the IEEE 1588
 * reference clock frequency + 2 ns) to compensate for input path and
 * synchronization delays. The adjustment time depends on the reference frequency
 * programmed and has to be adjusted based on the clock selected by the host.
 *
 * The 2 ns here is the maximum value for GPIO input delay.
 * There is one synchronizer in the Event input from GPIO path.
*/
#define PTP_EVENT_TS_COMP     (PTP_CLOCK_PERIOD_NSEC*3 + 2)

#define MAX_DP83TG721_PHY     5
#define MAX_TS_INFO_POOL      64
#define MAX_TXTS_POOL         64
#define MAX_EVENT_INFO_POOL   64

#define LIST_FOREACH(list, node) \
    for(node=(list)->next; node!=(list); node=node->next)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
typedef struct RegVal_s
{
    uint32_t reg;
    uint16_t val;
} RegVal;

typedef struct TsInfo_s
{
    uint32_t msgType;
    uint32_t seqId;
    uint64_t ts;
} TsInfo;

typedef struct ListNode_s
{
    struct ListNode_s *prev;
    struct ListNode_s *next;
} ListNode;

typedef struct TxTsWait_s
{
    ListNode node;
    uint32_t msgType;
    uint32_t seqId;
    uint64_t timeout;
} TxTsWait;

typedef struct TsReadyQ_s
{
    TsInfo tsInfoTable[MAX_TS_INFO_POOL];
    uint32_t size;
    uint16_t writeP;
    const char *name;
} TsReadyQ;

typedef struct EventTsInfo_s
{
    uint32_t index;
    uint32_t seqId;
    uint64_t ts;
} EventTsInfo;

typedef struct EventTsReadyQ_s
{
    EventTsInfo eventInfoTable[MAX_EVENT_INFO_POOL];
    uint32_t size;
    uint16_t writeP;
    uint16_t readP;
    uint32_t seqId;
} EventTsReadyQ;

typedef struct PhyRxTs_s {
    uint16_t nsLow;   /* ns[15:0] */
    uint16_t nsHigh;   /* overflow[1:0], ns[29:16] */
    uint16_t secLow;  /* sec[15:0] */
    uint16_t secHigh;  /* sec[31:16] */
    uint16_t seqId;   /* sequenceId[15:0] */
    uint16_t msgType; /* messageType[3:0], hash[11:0] */
} __attribute__((packed)) PhyRxTs;

typedef struct PhyTxTs_s {
    uint16_t nsLow;   /* ns[15:0] */
    uint16_t nsHigh;   /* overflow[1:0], ns[29:16] */
    uint16_t secLow;  /* sec[15:0] */
    uint16_t secHigh;  /* sec[31:16] */
} __attribute__((packed)) PhyTxTs;

typedef struct Dp83tg721Priv_s
{
    uint64_t tick;
    EnetPhy_Handle hPhy;
    bool isMaster;

    TsReadyQ rxTsReadyQ;
    TsReadyQ txTsReadyQ;

    ListNode txTsWaitList;
    ListNode txTsPoolList;
    TxTsWait txTsPoolData[MAX_TXTS_POOL];

    /* remember the last event timestamp */
    PhyTxTs eventData;
    EventTsReadyQ evTsReadyQ;

    uint8_t stsFrameEthHdr[14];
} Dp83tg721Priv;

typedef struct Timespec64_s
{
    int64_t sec; /* seconds */
    int32_t nsec; /* nanoseconds */
} Timespec64;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static bool Dp83tg721_isPhyDevSupported(EnetPhy_Handle hPhy,
                    const EnetPhy_Version *version);
static bool Dp83tg721_isMacModeSupported(EnetPhy_Handle hPhy, EnetPhy_Mii mii);
static int32_t Dp83tg721_config(EnetPhy_Handle hPhy,
                    const EnetPhy_Cfg *cfg, EnetPhy_Mii mii);
static void Dp83tg721_reset(EnetPhy_Handle hPhy);
static bool Dp83tg721_isResetComplete(EnetPhy_Handle hPhy);
static int32_t Dp83tg721_readExtReg(EnetPhy_Handle hPhy, uint32_t reg, uint16_t *val);
static int32_t Dp83tg721_writeExtReg(EnetPhy_Handle hPhy, uint32_t reg, uint16_t val);
static void Dp83tg721_printRegs(EnetPhy_Handle hPhy);
static void Dp83tg721_readStraps(EnetPhy_Handle hPhy);
static void Dp83tg721_chipInit(EnetPhy_Handle hPhy, Dp83tg721Priv *priv);
static void Dp83tg721_setBitsExtReg(EnetPhy_Handle hPhy, uint32_t reg, uint16_t mask);
static void Dp83tg721_clearBitsExtReg(EnetPhy_Handle hPhy, uint32_t reg, uint16_t mask);
static void Dp83tg721_configIntr(EnetPhy_Handle hPhy, bool intrEn);
static int32_t Dp83tg721_adjFreq(EnetPhy_Handle hPhy, int64_t ppb);
static int32_t Dp83tg721_adjPhase(EnetPhy_Handle hPhy, int64_t offset);
static int32_t Dp83tg721_getTxTs(EnetPhy_Handle hPhy, uint32_t domain,
                    uint32_t msgType, uint32_t seqId, uint64_t *ts64);
static int32_t Dp83tg721_getRxTs(EnetPhy_Handle hPhy, uint32_t domain,
                    uint32_t msgType, uint32_t seqId, uint64_t *ts64);
static int32_t Dp83tg721_setTime(EnetPhy_Handle hPhy, uint64_t ts64);
static int32_t Dp83tg721_getTime(EnetPhy_Handle hPhy, uint64_t *ts64);
static int32_t Dp83tg721_waitPtpTxTime(EnetPhy_Handle hPhy, uint32_t domain,
                    uint32_t msgType, uint32_t seqId);
static int32_t Dp83tg721_procStatusFrame(EnetPhy_Handle hPhy,
                    uint8_t *frame, uint32_t size, uint32_t *types);
static int32_t Dp83tg721_getStatusFrameEthHeader(EnetPhy_Handle hPhy,
                    uint8_t *ethhdr, uint32_t size);
static int32_t Dp83tg721_enablePtp(EnetPhy_Handle hPhy, bool on,
                    uint32_t srcMacStatusFrameType);
static int32_t Dp83tg721_tickDriver(EnetPhy_Handle hPhy);
static int32_t Dp83tg721_enableEventCapture(EnetPhy_Handle hPhy, uint32_t eventIdx,
                    bool falling, bool on);
static int32_t Dp83tg721_enableTriggerOutput(EnetPhy_Handle hPhy, uint32_t triggerIdx,
                    uint64_t start, uint64_t period, bool repeat);

static int32_t Dp83tg721_getEventTs(EnetPhy_Handle hPhy, uint32_t *eventIdx,
                    uint32_t *seqId, uint64_t *ts64);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

EnetPhy_Drv gEnetPhyDrvDp83tg721 =
{
    .name                = "Dp83tg721",
    .isPhyDevSupported   = Dp83tg721_isPhyDevSupported,
    .isMacModeSupported  = Dp83tg721_isMacModeSupported,
    .config              = Dp83tg721_config,
    .reset               = Dp83tg721_reset,
    .isResetComplete     = Dp83tg721_isResetComplete,
    .readExtReg          = Dp83tg721_readExtReg,
    .writeExtReg         = Dp83tg721_writeExtReg,
    .printRegs           = Dp83tg721_printRegs,
    .adjPtpFreq              = Dp83tg721_adjFreq,
    .adjPtpPhase             = Dp83tg721_adjPhase,
    .getPtpTime              = Dp83tg721_getTime,
    .setPtpTime              = Dp83tg721_setTime,
    .getPtpTxTime            = Dp83tg721_getTxTs,
    .getPtpRxTime            = Dp83tg721_getRxTs,
    .waitPtpTxTime           = Dp83tg721_waitPtpTxTime,
    .procStatusFrame         = Dp83tg721_procStatusFrame,
    .getStatusFrameEthHeader = Dp83tg721_getStatusFrameEthHeader,
    .enablePtp               = Dp83tg721_enablePtp,
    .tickDriver              = Dp83tg721_tickDriver,
    .enableEventCapture      = Dp83tg721_enableEventCapture,
    .enableTriggerOutput     = Dp83tg721_enableTriggerOutput,
    .getEventTs              = Dp83tg721_getEventTs,
};

/* PHY Device Attributes */
static Dp83tg721Priv gDp83tg721_Table[MAX_DP83TG721_PHY];

static uint8_t gGpioTable[GPIO_NUM_PINS] =
{
    /* Trigger pins */
    [START_TRIGGER_IDX] = LED_0,
    [START_TRIGGER_IDX+1] = CLKOUT,
    /* Capture pins */
    [START_EVENT_IDX] = STRP_1,
    [START_EVENT_IDX+1] = LED_1
};

static const RegVal gMasterRegsInit[] =
{
    { 0x405, 0x6C00 },
    { 0x8ad, 0x3c51 },
    { 0x894, 0x5df7 },
    { 0x8a0, 0x9e7  },
    { 0x8c0, 0x4000 },
    { 0x814, 0x4800 },
    { 0x80d, 0x2ebf },
    { 0x8c1, 0xb00  },
    { 0x87d, 0x001  },
    { 0x82e, 0x000  },
    { 0x837, 0x0f4  },
    { 0x8be, 0x200  },
    { 0x8c5, 0x4000 },
    { 0x8c7, 0x2000 },
    { 0x8b3, 0x05a  },
    { 0x8b4, 0x05a  },
    { 0x8b0, 0x202  },
    { 0x8b5, 0x0ea  },
    { 0x8ba, 0x2828 },
    { 0x8bb, 0x6828 },
    { 0x8bc, 0x028  },
    { 0x8bf, 0x000  },
    { 0x8b1, 0x014  },
    { 0x8b2, 0x008  },
    { 0x8ec, 0x000  },
    { 0x8c8, 0x003  },
    { 0x8be, 0x201  },
};

static const RegVal gSlaveRegsInit[] =
{
    { 0x405, 0x6C00 },
    { 0x8ad, 0x3c51 },
    { 0x894, 0x5df7 },
    { 0x8a0, 0x9e7  },
    { 0x8c0, 0x4000 },
    { 0x814, 0x4800 },
    { 0x80d, 0x2ebf },
    { 0x8c1, 0xb00  },
    { 0x87d, 0x001  },
    { 0x82e, 0x000  },
    { 0x837, 0x0f4  },
    { 0x8be, 0x200  },
    { 0x8c5, 0x4000 },
    { 0x8c7, 0x2000 },
    { 0x8b3, 0x05a  },
    { 0x8b4, 0x05a  },
    { 0x8b0, 0x202  },
    { 0x8b5, 0x0ea  },
    { 0x8ba, 0x2828 },
    { 0x8bb, 0x6828 },
    { 0x8bc, 0x028  },
    { 0x8bf, 0x000  },
    { 0x8b1, 0x014  },
    { 0x8b2, 0x008  },
    { 0x8ec, 0x000  },
    { 0x8c8, 0x003  },
    { 0x8be, 0x201  },
    { 0x56a, 0x5f40 },
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void Dp83tg721_initCfg(Dp83tg721_Cfg *cfg)
{
    /* No extended config parameters at the moment */
}

static void ListInit(ListNode *list)
{
    list->prev=list;
    list->next=list;
}

/* Push to the tail */
static void ListPush(ListNode *list, ListNode *node)
{
    list->prev->next=node;
    node->prev=list->prev;
    node->next=list;
    list->prev=node;
}

static void ListUnlink(ListNode *node)
{
    node->prev->next=node->next;
    node->next->prev=node->prev;
    node->next=NULL;
    node->prev=NULL;
}

static bool ListEmpty(ListNode *list)
{
    return (list->next==list);
}

/* Pop from the head */
static ListNode* ListPop(ListNode *list)
{
    ListNode *node = NULL;
    if (ListEmpty(list) == false)
    {
        node = list->next;
        ListUnlink(node);
    }
    return node;
}

static void InitPriv(Dp83tg721Priv *priv, EnetPhy_Handle hPhy)
{
    int32_t i;
    /* default of the status frame ethernet header */
    uint8_t ethHdr[14] = {
        0x01, 0x1B, 0x19, 0x00, 0x00, 0x00, /* Dest MAC */
        0x08, 0x00, 0x17, 0x0B, 0x6B, 0x0F, /* Src MAC */
        0x88, 0xF7 /* EtherType value for IEEE1588v2 */
    };

    memset(priv, 0, sizeof(Dp83tg721Priv));
    priv->isMaster = false;
    priv->hPhy = hPhy;
    hPhy->priv = priv;
    ListInit(&priv->txTsWaitList);
    ListInit(&priv->txTsPoolList);
    for (i = 0; i < MAX_TXTS_POOL; i++)
    {
        ListPush(&priv->txTsPoolList, &priv->txTsPoolData[i].node);
    }
    priv->txTsReadyQ.name = "TXQ";
    priv->txTsReadyQ.size = MAX_TS_INFO_POOL;
    priv->rxTsReadyQ.name = "RXQ";
    priv->rxTsReadyQ.size = MAX_TS_INFO_POOL;
    priv->evTsReadyQ.size = MAX_EVENT_INFO_POOL;

    memcpy(priv->stsFrameEthHdr, ethHdr, sizeof(ethHdr));
}

static Dp83tg721Priv *GetFreePriv(void)
{
    int32_t i;
    Dp83tg721Priv *priv = NULL;

    for (i = 0; i < MAX_DP83TG721_PHY; i++)
    {
        if (gDp83tg721_Table[i].hPhy == NULL)
        {
            priv = &gDp83tg721_Table[i];
            break;
        }
    }
    return priv;
}

static bool Dp83tg721_isPhyDevSupported(EnetPhy_Handle hPhy,
                    const EnetPhy_Version *version)
{
    bool supported = false;

    if ((version->oui == DP83TG721_OUI) &&
        (version->model == DP83TG721_MODEL) &&
        (version->revision == DP83TG721_REV))
    {
        supported = true;
    }
    return supported;
}

static bool Dp83tg721_isMacModeSupported(EnetPhy_Handle hPhy, EnetPhy_Mii mii)
{
    bool supported = false;

    switch (mii)
    {
        case ENETPHY_MAC_MII_RGMII:
        case ENETPHY_MAC_MII_SGMII:
            supported = true;
            break;
        case ENETPHY_MAC_MII_MII:
        case ENETPHY_MAC_MII_GMII:
        default:
            supported = false;
            break;
    }

    return supported;
}

static void Dp83tg721_setLoopbackCfg(EnetPhy_Handle hPhy, bool enable)
{
    bool complete;
    int32_t status;
    uint16_t val;

    ENETTRACE_DBG("PHY %u: %s loopback\n", hPhy->addr, enable ? "enable" : "disable");

    status = EnetPhy_readReg(hPhy, PHY_BMCR, &val);
    if (status == ENETPHY_SOK)
    {
        if (enable)
        {
            //xMII Loopback Mode
            val |= BMCR_LOOPBACK;
        }
        else
        {
            //Normal Mode
            val &= ~BMCR_LOOPBACK;
        }
        /* Specific predefined loopback configuration values are required for
         * normal mode or loopback mode */
        EnetPhy_writeReg(hPhy, PHY_BMCR, val);

        /* Software restart is required after changing LOOPCR register */
        Dp83tg721_reset(hPhy);

        do
        {
            complete = Dp83tg721_isResetComplete(hPhy);
        } while (complete == false);
    }
}

static int32_t Dp83tg721_config(EnetPhy_Handle hPhy,
                const EnetPhy_Cfg *cfg, EnetPhy_Mii mii)
{
    uint16_t rxIntDelay = 1U;    //Enable RX Clock Shift
    uint16_t txIntDelay = 1U;    //Enable TX Clock Shift
    uint16_t rgmiiDelay = 0U;
    uint16_t value = 0;
    bool enableAutoNeg = true;
    EnetPhy_State *state = &hPhy->state;
    int32_t status = ENETPHY_SOK;
    Dp83tg721Priv *priv = hPhy->priv;

    if (priv == NULL)
    {
        priv = GetFreePriv();
        if (priv != NULL)
        {
            InitPriv(priv, hPhy);
        }
        else
        {
            ENETTRACE_ERR("PHY %u: GetFreePriv error\n", hPhy->addr);
            status = CSL_EALLOC-6;
        }
    }

    if (status == ENETPHY_SOK)
    {
        /* Set Parameters for Automotive PHYs - Fixed Speed */
        state->phyLinkCaps = ENETPHY_LINK_CAP_FD1000; // 1000Mbit/s - Full Duplex

        Dp83tg721_readStraps(hPhy);

        Dp83tg721_chipInit(hPhy, priv);

        if (mii == ENETPHY_MAC_MII_SGMII)
        {
            ENETTRACE_DBG("PHY %u: Enabling SGMII Mode\n", hPhy->addr);

            status = Dp83tg721_readExtReg(hPhy, SGMII_CTRL_1, &value);

            value |= SGMII_EN;
            if(enableAutoNeg)
                value |= SGMII_AUTO_NEG_EN;
            else
                value &= ~SGMII_AUTO_NEG_EN;

            if (status == ENETPHY_SOK)
                Dp83tg721_writeExtReg(hPhy, SGMII_CTRL_1, value);
        }
        //Enable RGMII Interface
        else if (mii == ENETPHY_MAC_MII_RGMII)
        {
            Dp83tg721_writeExtReg(hPhy, RGMII_CTRL, 0x0128U);
            if (rxIntDelay == 0)
                rgmiiDelay &= ~RGMII_RX_SHIFT;
            else
                rgmiiDelay |= RGMII_RX_SHIFT;
            if (txIntDelay == 0)
                rgmiiDelay &= ~RGMII_TX_SHIFT;
            else
                rgmiiDelay |= RGMII_TX_SHIFT;
            Dp83tg721_setBitsExtReg(hPhy, RGMII_DELAY_CTRL, rgmiiDelay);
        }
        else
        {
            ENETTRACE_DBG("PHY %u: Disabling SRGMII Mode, disable Autoneg\n",
                          hPhy->addr);

            status = Dp83tg721_readExtReg(hPhy, SGMII_CTRL_1, &value);
            value &= ~(SGMII_EN | SGMII_AUTO_NEG_EN);
            if (status == ENETPHY_SOK)
                Dp83tg721_writeExtReg(hPhy, SGMII_CTRL_1, value);
        }
    }

    if (status == ENETPHY_SOK)
    {
        Dp83tg721_configIntr(hPhy, false);
        Dp83tg721_setLoopbackCfg(hPhy, cfg->loopbackEn);
    }

    return status;
}

static void Dp83tg721_reset(EnetPhy_Handle hPhy)
{
    /* Global software reset */
    ENETTRACE_DBG("PHY %u: global soft-reset\n", hPhy->addr);
    EnetPhy_rmwReg(hPhy, MII_REG_1F, SW_RESET, SW_RESET);
}

static void Dp83tg721_resetHw(EnetPhy_Handle hPhy)
{
    /* Global hardware reset */
    ENETTRACE_DBG("PHY %u: global hard-reset\n", hPhy->addr);
    EnetPhy_rmwReg(hPhy, MII_REG_1F, HW_RESET, HW_RESET);
}

static bool Dp83tg721_isResetComplete(EnetPhy_Handle hPhy)
{
    int32_t status;
    uint16_t val;
    bool complete = false;

    /* Reset is complete when RESET bits have self-cleared */
    status = EnetPhy_readReg(hPhy, MII_REG_1F, &val);
    if (status == ENETPHY_SOK)
    {
        complete = ((val & (SW_RESET | HW_RESET)) == 0U);
    }

    ENETTRACE_DBG("PHY %u: global reset is %s complete\n",
                  hPhy->addr, complete ? "" : "not");

    return complete;
}

static int32_t ExtRegToDevad(uint16_t reg, uint16_t *accessReg, uint16_t *devad)
{
    int32_t status = ENETPHY_SOK;
    /* Registers range:
     * 0x000 - 0x0EFD: MMD1F
     * 0x1000 - 0x1904: MMD1
     * 0x3000 - 0x390D: MMD3
     * 0x7000 - 0x7200: MMD7
     */
    if ((reg >= 0) && (reg <= 0xEFD))
    {
        *devad = 0x1F;
        *accessReg = reg;
    }
    else if (((reg >= 0x1000) && (reg <= 0x1904)) ||
             ((reg >= 0x3000) && (reg <= 0x390D)) ||
             ((reg >= 0x7000) && (reg <= 0x7200)))
    {
        uint32_t mostSigNibble = reg >> 12;
        /*
        * For MMD1/3/7, most significant nibble of the register address
        * is used to denote the respective MMD space.
        * This should be ignored during actual register access operation.
        * For example to access register 0x1904 use 0x0904 as the register
        * address and x01 as the MMD.
        */
        *devad = mostSigNibble;
        *accessReg = reg & 0x0FFF;
    }
    else
    {
        status = ENETPHY_EBADARGS;
    }

    return status;
}

static uint32_t Dp83tg721_accessExtReg(EnetPhy_Handle hPhy, uint32_t reg)
{
    int32_t status;
    uint16_t devad;
    uint16_t accessReg;

    status = ExtRegToDevad((uint16_t)reg, &accessReg, &devad);
    if (status == ENETPHY_SOK)
    {
        status = EnetPhy_writeReg(hPhy, PHY_MMD_CR, devad | MMD_CR_ADDR);
    }
    if (status == ENETPHY_SOK)
    {
        status = EnetPhy_writeReg(hPhy, PHY_MMD_DR, accessReg);
    }
    if (status == ENETPHY_SOK)
    {
        status = EnetPhy_writeReg(hPhy, PHY_MMD_CR, devad | MMD_CR_DATA_NOPOSTINC);
    }
    return status;
}

static int32_t Dp83tg721_readExtReg(EnetPhy_Handle hPhy, uint32_t reg, uint16_t *val)
{
    int32_t status;

    status = Dp83tg721_accessExtReg(hPhy, reg);
    if (status == ENETPHY_SOK)
    {
        status = EnetPhy_readReg(hPhy, PHY_MMD_DR, val);
    }

    ENETTRACE_VERBOSE_IF(status == ENETPHY_SOK,
                         "PHY %u: read reg %u val 0x%04x\n",
                         hPhy->addr, reg, *val);

    ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                     "PHY %u: failed to read reg %u\n",
                     hPhy->addr, reg);

    return status;
}

static int32_t Dp83tg721_writeExtReg(EnetPhy_Handle hPhy, uint32_t reg, uint16_t val)
{
    int32_t status;

    ENETTRACE_VERBOSE("PHY %u: write %u val 0x%04x\n",
                      hPhy->addr, reg, val);

    status = Dp83tg721_accessExtReg(hPhy, reg);
    if (status == ENETPHY_SOK)
    {
        status = EnetPhy_writeReg(hPhy, PHY_MMD_DR, val);
    }

    ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                     "PHY %u: failed to write reg %u val 0x%04x\n",
                     hPhy->addr, reg, val);
    return status;
}

static void Dp83tg721_setBitsExtReg(EnetPhy_Handle hPhy, uint32_t reg, uint16_t mask)
{
    uint16_t value;
    int32_t status;

    status = Dp83tg721_readExtReg(hPhy, reg, &value);
    if (status == ENETPHY_SOK)
    {
        SET_BIT(mask, value);
        Dp83tg721_writeExtReg(hPhy, reg, value);
    }
}

static void Dp83tg721_clearBitsExtReg(EnetPhy_Handle hPhy, uint32_t reg, uint16_t mask)
{
    uint16_t value;
    int32_t status;

    status = Dp83tg721_readExtReg(hPhy, reg, &value);
    if (status == ENETPHY_SOK)
    {
        CLEAR_BIT(mask, value);
        Dp83tg721_writeExtReg(hPhy, reg, value);
    }
}

static void Dp83tg721_readStraps(EnetPhy_Handle hPhy)
{
    uint16_t strap;
    int32_t status;
    Dp83tg721Priv *priv = hPhy->priv;

    if (priv)
    {
        status = Dp83tg721_readExtReg(hPhy, SOR_VECTOR_1, &strap);
        if (status == ENETPHY_SOK)
        {
            ENETTRACE_DBG("Strap is 0x%X\n", strap);
            if (strap & MASTER_MODE)
            {
                priv->isMaster = true;
                ENETTRACE_DBG("Strap: Master Mode enabled\n");
            }
            else
            {
                priv->isMaster = false;
                ENETTRACE_DBG("Strap: Slave Mode enabled\n");
            }
        }
    }
    else
    {
        ENETTRACE_ERR("No phy priv\n");
    }
}

static void Dp83tg721_writeSeq(EnetPhy_Handle hPhy, const RegVal *regVals, int32_t size)
{
    int32_t i;
    for (i = 0; i < size; i++)
    {
        Dp83tg721_writeExtReg(hPhy, regVals[i].reg, regVals[i].val);
    }
}

static void Dp83tg721_chipInit(EnetPhy_Handle hPhy, Dp83tg721Priv *priv)
{
    bool complete = false;

    Dp83tg721_resetHw(hPhy);

    /* Don't let PHY start link-up procedure */
    Dp83tg721_writeExtReg(hPhy, 0x0573U, 0x0101U);

    if (priv->isMaster)
    {
        /* Set specific value in BMSR register */
        EnetPhy_writeReg(hPhy, PHY_BMSR, 0x0940U);
        EnetPhy_writeReg(hPhy, PHY_BMSR, 0x0140U);

        /* Set Master mode */
        Dp83tg721_writeExtReg(hPhy, PMA_PMD_CONTROL, 0xC001U);
        Dp83tg721_writeSeq(hPhy, gMasterRegsInit,
            sizeof(gMasterRegsInit)/sizeof(gMasterRegsInit[0]));
    }
    else
    {
        /* Set Slave mode */
        Dp83tg721_writeExtReg(hPhy, PMA_PMD_CONTROL, 0x8001U);
        Dp83tg721_writeSeq(hPhy, gSlaveRegsInit,
            sizeof(gSlaveRegsInit)/sizeof(gSlaveRegsInit[0]));
    }

    /* Enable the PHY */
    EnetPhy_writeReg(hPhy, LPS_CFG3, 0x0001U);

    /* Do a software reset to restart the PHY with the updated values */
    Dp83tg721_reset(hPhy);
    do
    {
        complete = Dp83tg721_isResetComplete(hPhy);
    } while (complete == false);

    /* Let the PHY start link-up procedure */
    Dp83tg721_writeExtReg(hPhy, 0x0573U, 0x0001U);

    /* Start send-s detection during link-up sequence */
    Dp83tg721_writeExtReg(hPhy, 0x056AU, 0x5F41U);
}

static void Dp83tg721_configIntr(EnetPhy_Handle hPhy, bool intrEn)
{
    uint16_t regVal;
    int32_t status;

    if (intrEn)
    {
        ENETTRACE_DBG("PHY %u: Enable interrupts\n", hPhy->addr);
        status = EnetPhy_readReg(hPhy, MII_REG_12, &regVal);
        if (status == ENETPHY_SOK)
        {
            regVal |= (TRAINING_DONE_INT_EN | ESD_EVENT_INT_EN |
                       LINK_STAT_INT_EN | ENERGY_DET_INT_EN);
            EnetPhy_writeReg(hPhy, MII_REG_12, regVal);
            status = EnetPhy_readReg(hPhy, MII_REG_13, &regVal);
            if (status == ENETPHY_SOK)
            {
                regVal |= (OVERTEMP_INT_EN | OVERVOLTAGE_INT_EN | UNDERVOLTAGE_INT_EN);
                EnetPhy_writeReg(hPhy, MII_REG_13, regVal);
                status = EnetPhy_readReg(hPhy, MII_REG_18, &regVal);
                if (status == ENETPHY_SOK)
                {
                    regVal |= (LPS_INT_EN | WUR_INT_EN | POR_DONE_INT_EN);
                    EnetPhy_writeReg(hPhy, MII_REG_18, regVal);
                }
            }
        }
    }
    else
    {
        ENETTRACE_DBG("PHY %u: Disable interrupts\n", hPhy->addr);
        EnetPhy_writeReg(hPhy, MII_REG_12, 0U);
        EnetPhy_writeReg(hPhy, MII_REG_13, 0U);
        EnetPhy_writeReg(hPhy, MII_REG_18, 0U);
    }
}

static void Dp83tg721_printRegs(EnetPhy_Handle hPhy)
{
    uint32_t phyAddr = hPhy->addr;
    uint16_t val;

    EnetPhy_readReg(hPhy, PHY_BMCR, &val);
    EnetUtils_printf("PHY %u: BMCR    = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_BMSR, &val);
    EnetUtils_printf("PHY %u: BMSR    = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_PHYIDR1, &val);
    EnetUtils_printf("PHY %u: PHYIDR1 = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_PHYIDR2, &val);
    EnetUtils_printf("PHY %u: PHYIDR2 = 0x%04x\n", phyAddr, val);
    Dp83tg721_readExtReg(hPhy, SGMII_CTRL_1, &val);
    EnetUtils_printf("PHY %u: SGMII_CTRL = 0x%04x\n", hPhy->addr, val);
    Dp83tg721_readExtReg(hPhy, RGMII_CTRL, &val);
    EnetUtils_printf("PHY %u: RGMII_CTRL = 0x%04x\n", hPhy->addr, val);
    Dp83tg721_readExtReg(hPhy, RGMII_DELAY_CTRL, &val);
    EnetUtils_printf("PHY %u: RGMII_DELAY_CTRL = 0x%04x\n", hPhy->addr, val);
    Dp83tg721_readExtReg(hPhy, A2D_REG_48, &val);
    EnetUtils_printf("PHY %u: RGMII_DELAY_TX_RX = 0x%04x\n", hPhy->addr, val);
    Dp83tg721_readExtReg(hPhy, PMA_PMD_CONTROL, &val);
    EnetUtils_printf("PHY %u: REG_MasterSlave = 0x%04x\n", hPhy->addr, val);
}

static int32_t Dp83tg721_adjFreq(EnetPhy_Handle hPhy, int64_t ppb)
{
    uint64_t rate;
    int32_t negAdj = 0;
    uint16_t hi;
    uint16_t lo;

    if (ppb < 0) {
        negAdj = 1;
        ppb = -ppb;
    }
    /*
     * The Rate Control value is in units of 2^32 ns:
     * rate = ppb * PTP_CLOCK_PERIOD_NSEC * 2^32 / 10^9
     * = ppb * (10^9 / PTP_CLOCK_FREQ_HZ) * 2^32 / 10^9
     * = (ppb * 2^32) / PTP_CLOCK_FREQ_HZ
     */
    rate = (ppb << 32) / PTP_CLOCK_FREQ_HZ;

    hi = ENCODE_BIGFIELD(PTP_RATE_HI, rate >> 16);
    if (negAdj)
        hi |= PTP_RATE_DIR;

    lo = rate & 0xffff;

    Dp83tg721_writeExtReg(hPhy, PTP_RATEH, hi);
    Dp83tg721_writeExtReg(hPhy, PTP_RATEL, lo);

    return ENETPHY_SOK;
}

static uint64_t DivU64Rem(uint64_t dividend, uint32_t divisor, uint32_t *remainder)
{
    *remainder = dividend % divisor;
    return dividend / divisor;
}

static void NsecToTimespec64(const int64_t nsec, Timespec64 *ts)
{
    uint32_t rem;

    if (nsec > 0)
    {
        ts->sec = DivU64Rem(nsec, SEC_NSEC, &rem);
        ts->nsec = rem;
    }
    else if (nsec < 0)
    {
        /*
         * With negative times, sec points to the earlier
         * second, and nsec counts the nanoseconds since
         * then, so nsec is always a positive number.
         */
        ts->sec = -DivU64Rem(-nsec - 1, SEC_NSEC, &rem) - 1;
        ts->nsec = SEC_NSEC - rem - 1;
    }
}

static int32_t TdrWrite(EnetPhy_Handle hPhy, int64_t nsec, uint16_t cmd)
{
    Timespec64 ts;

    NsecToTimespec64(nsec, &ts);

    Dp83tg721_writeExtReg(hPhy, PTP_TDR, ts.nsec & 0xffff);/* ns[15:0]  */
    Dp83tg721_writeExtReg(hPhy, PTP_TDR, ts.nsec >> 16);   /* ns[31:16] */
    Dp83tg721_writeExtReg(hPhy, PTP_TDR, ts.sec & 0xffff); /* sec[15:0] */
    Dp83tg721_writeExtReg(hPhy, PTP_TDR, ts.sec >> 16);    /* sec[31:16]*/

    Dp83tg721_writeExtReg(hPhy, PTP_CTL, cmd);

    return ENETPHY_SOK;
}

static int32_t Dp83tg721_adjPhase(EnetPhy_Handle hPhy, int64_t offset)
{
    return TdrWrite(hPhy, offset+(int64_t)ADJTIME_FIX, PTP_STEP_CLK);
}

static uint32_t ExtRead(EnetPhy_Handle hPhy, uint32_t reg)
{
    uint16_t val = 0;
    Dp83tg721_readExtReg(hPhy, reg, &val);
    return (uint32_t)val;
}

static int32_t Dp83tg721_getTime(EnetPhy_Handle hPhy, uint64_t *ts64)
{
    uint32_t val[4];
    uint64_t sec = 0;
    uint64_t nsec = 0;

    Dp83tg721_writeExtReg(hPhy, PTP_CTL, PTP_RD_CLK);

    val[0] = ExtRead(hPhy, PTP_TDR); /* ns[15:0] */
    val[1] = ExtRead(hPhy, PTP_TDR); /* ns[31:16] */
    val[2] = ExtRead(hPhy, PTP_TDR); /* sec[15:0] */
    val[3] = ExtRead(hPhy, PTP_TDR); /* sec[31:16] */

    nsec = val[0] | (val[1] << 16);
    sec  = val[2] | (val[3] << 16);

    *ts64 = sec * SEC_NSEC + nsec;

    return ENETPHY_SOK;
}

static int32_t Dp83tg721_setTime(EnetPhy_Handle hPhy, uint64_t ts64)
{
    return TdrWrite(hPhy, ts64, PTP_LOAD_CLK);
}

static bool IsLittleEndian(void)
{
    int32_t check = 1;
    char *ptr = (char*)&check;
    bool result = false;

    if((*ptr) == 1)
    {
        result = true;
    }

    return result;
}

static void Dp83tg721_enableStatusFrames(EnetPhy_Handle hPhy, bool on,
                    uint32_t srcMacStatusFrameType)
{
    uint16_t cfg0 = 0;
    uint16_t ver;
    Dp83tg721Priv *priv = hPhy->priv;

    if (on == true)
    {
        cfg0 = PSF_EVNT_EN | PSF_RXTS_EN | PSF_TXTS_EN;
        if (IsLittleEndian() == true)
        {
            cfg0 |= PSF_ENDIAN;
        }
        /* Phy Status Frame Mac Source Address:
         * 0x0 = Use Mac Address [08 00 17 0B 6B 0F]
         * 0x1 = Use Mac Address [08 00 17 00 00 00]
         * 0x2 = Use Mac Multicast Dest Address
         * 0x3 = Use Mac Address [00 00 00 00 00 00].
         * We will not support srcMacStatusFrameType = 0x2 because the source MAC
         * in the priv->stsFrameEthHdr will be undefined unless we support
         * a new API that allows the app to set the multicast destination address.
         * Most apps will not require this.
         */
        if ((srcMacStatusFrameType == 0x0) ||
            (srcMacStatusFrameType == 0x1) ||
            (srcMacStatusFrameType == 0x3))
        {
            cfg0 |= ENCODE_BIGFIELD(MAC_SRC_ADD, srcMacStatusFrameType);
            if (srcMacStatusFrameType == 0x1)
            {
                memset(&priv->stsFrameEthHdr[9], 0, 3);
            }
            else if (srcMacStatusFrameType == 0x3)
            {
                memset(&priv->stsFrameEthHdr[6], 0, 6);
            }
        }
    }

    ver = ENCODE_BIGFIELD(VERSIONPTP, PTP_VERSION);

    Dp83tg721_writeExtReg(hPhy, PSF_CFG0, cfg0);
    Dp83tg721_writeExtReg(hPhy, PSF_CFG1, ver);
}

static int32_t Dp83tg721_enablePtp(EnetPhy_Handle hPhy, bool on,
                    uint32_t srcMacStatusFrameType)
{
    uint16_t txcfg0;
    uint16_t rxcfg0;
    // uint16_t txcfg1;
    // uint16_t rxcfg1;

    /* Configuration for using free running 250 MHz from PLL */

    /* Step 1: Toggle PTP_RESET 1 -> 0 to perform PTP reset */
    Dp83tg721_setBitsExtReg(hPhy, PTP_CTL, PTP_RESET);
    Dp83tg721_clearBitsExtReg(hPhy, PTP_CTL, PTP_RESET);

    /* After reset the PTP is disabled as default */
    if (on == true)
    {
        /* Step 2: Enable PTP by set PTP_ENABLE */
        Dp83tg721_setBitsExtReg(hPhy, PTP_CTL, PTP_ENABLE);

        /* Step 3: After reset, default PTP_CLKSRC is set to 250 MHz PLL,
        * and PTP Clock Source Period is set to 4.
        * We don't have to configure this register */

        /* Step 4: After reset, default PTP_RATEH_ACC_ONLY of all bits are set to 0:
        * Disable Accumulator Mode
        * We don't have to configure this register */

        /* Step 5: (Optional) Required when Clock Output synchronized to
        * wall clock is needed on a pin */
        Dp83tg721_setBitsExtReg(hPhy, PTP_PLL_EN_CTL, MR_PTP_PLL_PTP_PLL_EN);

        Dp83tg721_enableStatusFrames(hPhy, true, srcMacStatusFrameType);

        txcfg0 = ENCODE_BIGFIELD(TX_PTP_VER, PTP_VERSION) | TX_L2_EN | TX_TS_EN;
        rxcfg0 = ENCODE_BIGFIELD(RX_PTP_VER, PTP_VERSION) | RX_L2_EN | RX_TS_EN;

        Dp83tg721_writeExtReg(hPhy, PTP_TXCFG0, txcfg0);
        Dp83tg721_writeExtReg(hPhy, PTP_RXCFG0, rxcfg0);

		#if DP83TG721_ONLY_TIMESTAMP_PTP_EVENTS
        /* Timestamp only PTP events, not all PTP packets */
        Dp83tg721_writeExtReg(hPhy, PTP_TXCFG1, 0x0800);
        Dp83tg721_writeExtReg(hPhy, PTP_RXCFG1, 0x0800);
		#endif
    }

    return ENETPHY_SOK;
}

static uint64_t Dp83tg721_regTimeToNsec(uint16_t nsLow, uint16_t nsHigh,
                        uint16_t secLow, uint16_t secHigh)
{
    uint32_t sec;
    uint64_t ns;

    sec = secLow;
    sec |= secHigh << 16;

    ns = nsLow;
    ns |= (nsHigh & 0x3fff) << 16;
    ns += ((uint64_t)sec) * SEC_NSEC;

    return ns;
}

static void UpdateTsReadyQ(TsReadyQ *readyQ, TsInfo *newTs, uint32_t phyAddr)
{
    TsInfo *oldTs = &readyQ->tsInfoTable[readyQ->writeP];
    if (oldTs->ts)
    {
        ENETTRACE_DBG("PHY %u: Drop TS in %s: msgType=%d, seqId=%d\n",
                      phyAddr, readyQ->name, oldTs->msgType, oldTs->seqId);
    }
    memcpy(oldTs, newTs, sizeof(TsInfo));
    readyQ->writeP++;
    if (readyQ->writeP == readyQ->size)
    {
        readyQ->writeP = 0;
    }
}

static uint64_t LookupTsFromReadyQ(TsReadyQ *readyQ, uint32_t msgType, uint32_t seqId)
{
    int32_t i;
    uint64_t ts = 0;

    for (i = 0; i < readyQ->size; i++)
    {
        if ((readyQ->tsInfoTable[i].msgType == msgType) &&
            (readyQ->tsInfoTable[i].seqId == seqId))
        {
            ts = readyQ->tsInfoTable[i].ts;
            memset(&readyQ->tsInfoTable[i], 0, sizeof(TsInfo));
            break;
        }
    }
    return ts;
}

static int32_t Dp83tg721_getTs(EnetPhy_Handle hPhy, uint32_t domain, uint32_t msgType,
                               uint32_t seqId, uint64_t *ts64, bool tx)
{
    int32_t status = ENETPHY_SOK;
    Dp83tg721Priv *priv = hPhy->priv;

    if ((priv == NULL) || (ts64 == NULL))
    {
        status = ENETPHY_EBADARGS;
    }
    else
    {
        TsReadyQ *readyQ = &priv->rxTsReadyQ;
        if (tx)
            readyQ = &priv->txTsReadyQ;

        *ts64 = LookupTsFromReadyQ(readyQ, msgType, seqId);
        if ((*ts64) == 0)
        {
            status = CSL_EALLOC-6;
        }
    }

    return status;
}

static int32_t Dp83tg721_getTxTs(EnetPhy_Handle hPhy, uint32_t domain,
                uint32_t msgType, uint32_t seqId, uint64_t *ts64)
{
    return Dp83tg721_getTs(hPhy, domain, msgType, seqId, ts64, true);
}

static int32_t Dp83tg721_getRxTs(EnetPhy_Handle hPhy, uint32_t domain,
                uint32_t msgType, uint32_t seqId, uint64_t *ts64)
{
    return Dp83tg721_getTs(hPhy, domain, msgType, seqId, ts64, false);
}

static uint64_t PhyToTxTs(PhyTxTs *p)
{
    uint64_t ns;
    uint32_t sec;

    sec = p->secLow;
    sec |= p->secHigh << 16;

    ns = p->nsLow;
    ns |= (p->nsHigh & 0x3fff) << 16;
    ns += ((uint64_t)sec) * SEC_NSEC;

    return ns;
}

static void PhyToRxTs(PhyRxTs *p, TsInfo *rxts)
{
    uint32_t sec;

    sec = p->secLow;
    sec |= p->secHigh << 16;

    rxts->ts = p->nsLow;
    rxts->ts |= (p->nsHigh & 0x3fff) << 16;
    rxts->ts += ((uint64_t)sec) * SEC_NSEC;
    rxts->seqId = p->seqId;
    rxts->msgType = (p->msgType >> 12) & 0xf;
}

static void DecodeRxTs(Dp83tg721Priv *priv, PhyRxTs *phyRxTs)
{
    uint8_t overflow;
    TsInfo tsInfo;

    overflow = (phyRxTs->nsHigh >> 14) & 0x3;
    if (overflow)
        ENETTRACE_ERR("rx timestamp queue overflow, count %d\n", overflow);

    PhyToRxTs(phyRxTs, &tsInfo);
    UpdateTsReadyQ(&priv->rxTsReadyQ, &tsInfo, priv->hPhy->addr);
}

static bool TsExpired(uint64_t now, uint64_t timeout)
{
    bool result = false;

    if (now >= timeout)
    {
        result = true;
    }

    return result;
}

static void DecodeTxTs(Dp83tg721Priv *priv, PhyTxTs *phyTxTs)
{
    uint8_t overflow;
    // uint64_t ns;
    TxTsWait *wait;

    wait = (TxTsWait *)ListPop(&priv->txTsWaitList);
    if (wait != NULL)
    {
        overflow = (phyTxTs->nsHigh >> 14) & 0x3;
        if (overflow)
        {
            ENETTRACE_ERR("tx timestamp queue overflow, count %d\n", overflow);
            while (wait != NULL)
            {
                ListPush(&priv->txTsPoolList, &wait->node);
                wait = (TxTsWait *)ListPop(&priv->txTsWaitList);
            }
        }
        else
        {
            /* Remove all the expired timestamp */
            while (wait != NULL)
            {
                if (TsExpired(priv->tick, wait->timeout) == false)
                {
                    TsInfo tsInfo;
                    tsInfo.msgType = wait->msgType;
                    tsInfo.seqId = wait->seqId;
                    tsInfo.ts = PhyToTxTs(phyTxTs);

                    UpdateTsReadyQ(&priv->txTsReadyQ, &tsInfo, priv->hPhy->addr);
                    ListPush(&priv->txTsPoolList, &wait->node);
                    break;
                }
                else
                {
                    ListPush(&priv->txTsPoolList, &wait->node);
                    wait = (TxTsWait *)ListPop(&priv->txTsWaitList);
                }
            }
        }
    }
    else
    {
        ENETTRACE_ERR("PHY %u: recv txts, txWaitList empty\n", priv->hPhy->addr);
    }
}

static void UpdateEventTsReadyQ(EventTsReadyQ *readyQ, EventTsInfo *newEv,
                    uint32_t phyAddr)
{
    /* HW does not support a seqId for an event, we add it to allow user app to
     * detect if the event is lost because of queue full.
     * When the queue is full, user must read all the existing events,
     * otherwise the new event will be dropped. */
    EventTsInfo *writeEv = &readyQ->eventInfoTable[readyQ->writeP];
    if (writeEv->ts)
    {
        ENETTRACE_DBG("PHY %u: Drop Event TS: seqId=%d\n", phyAddr, writeEv->seqId);
    }
    else
    {
        newEv->seqId = readyQ->seqId;
        memcpy(writeEv, newEv, sizeof(EventTsInfo));
        readyQ->writeP++;
        if (readyQ->writeP == readyQ->size)
        {
            readyQ->writeP = 0;
        }
    }
    /* seqId will help to detect if the event is dropped due to the queue full */
    readyQ->seqId++;
}

static uint16_t EventNumToEventStatus(int eventNum)
{
    return 1 << (eventNum * 2);
}

static int32_t DecodeEvent(Dp83tg721Priv *priv,
                void *data, int32_t len, uint16_t ests)
{
    PhyTxTs *phyTxTs;
    int32_t parsed;
    int32_t words = DECODE_BIGFIELD(EVNT_TS_LEN, ests);
    uint16_t extStatus = 0;
    int32_t i;
    int32_t result = len;
    EventTsInfo event;

    /* calculate length of the event timestamp status message */
    if (ests & MULT_EVNT)
    {
        parsed = (words + 2) * sizeof(uint16_t);
    }
    else
    {
        parsed = (words + 1) * sizeof(uint16_t);
    }

    /* check if enough data is available */
    if (len >= parsed)
    {
        if (ests & MULT_EVNT)
        {
            extStatus = *(uint16_t *) data;
            data += sizeof(extStatus);
        }

        phyTxTs = data;

        /* This switch without break is intention */
        switch (words)
        {
        case 3:
            priv->eventData.secHigh = phyTxTs->secHigh;
        case 2:
            priv->eventData.secLow = phyTxTs->secLow;
        case 1:
            priv->eventData.nsHigh = phyTxTs->nsHigh;
        case 0:
            priv->eventData.nsLow = phyTxTs->nsLow;
        }

        if (extStatus == 0)
        {
            extStatus = EventNumToEventStatus(DECODE_BIGFIELD(EVNT_NUM, ests));
        }

        event.ts = PhyToTxTs(&priv->eventData);

        /* Compensate for input path and synchronization delays */
        event.ts -= PTP_EVENT_TS_COMP;

        for (i = 0; i < NUM_EVENTS; i++)
        {
            if (extStatus & EventNumToEventStatus(i))
            {
                event.index = i;
                UpdateEventTsReadyQ(&priv->evTsReadyQ, &event, priv->hPhy->addr);
            }
        }
        result = parsed;
    }

    return result;
}

static int32_t Dp83tg721_procStatusFrame(EnetPhy_Handle hPhy,
                    uint8_t *frame, uint32_t frameSize, uint32_t *types)
{
    PhyRxTs *phyRxTs;
    PhyTxTs *phyTxTs;
    uint8_t *ptr;
    int32_t len;
    int32_t size;
    uint16_t ests;
    uint16_t type;
    int32_t status = ENETPHY_SOK;
    Dp83tg721Priv * priv = hPhy->priv;

    if ((frame == NULL) || (priv == NULL) || (types == NULL))
    {
        status = ENETPHY_EBADARGS;
    }
    else if ((frameSize <= (sizeof(priv->stsFrameEthHdr) + 2)) ||
            (memcmp(frame, priv->stsFrameEthHdr, sizeof(priv->stsFrameEthHdr))))
    {
        status = ENETPHY_EINVALIDPARAMS;
    }
    else
    {
        /* Skip 2 first bytes: transportSpecific messageType & versionPTP */
        ptr = frame + sizeof(priv->stsFrameEthHdr) + 2;
        len = frameSize - (sizeof(priv->stsFrameEthHdr) + 2);

        *types = 0;
        for (; len > sizeof(type); len -= size)
        {
            type = *(uint16_t *)ptr;
            ests = type & 0x0fff;
            type = type & 0xf000;
            len -= sizeof(type);
            ptr += sizeof(type);

            if ((PSF_RX == type) && (len >= sizeof(PhyRxTs)))
            {
                phyRxTs = (PhyRxTs *)ptr;
                DecodeRxTs(priv, phyRxTs);
                size = sizeof(PhyRxTs);
                *types |= (1U << 1);
            }
            else if ((PSF_TX == type) && (len >= sizeof(PhyTxTs)))
            {
                phyTxTs = (PhyTxTs *)ptr;
                DecodeTxTs(priv, phyTxTs);
                size = sizeof(PhyTxTs);
                *types |= (1U << 1);
            }
            else if (PSF_EVNT == type)
            {
                size = DecodeEvent(priv, ptr, len, ests);
                *types |=  (1U << 2);
            }
            else
            {
                size = 0;
                break;
            }
            ptr += size;
        }
    }

    return status;
}

static int32_t Dp83tg721_waitPtpTxTime(EnetPhy_Handle hPhy, uint32_t domain,
                                       uint32_t msgType, uint32_t seqId)
{
    int32_t status = ENETPHY_SOK;
    Dp83tg721Priv * priv = hPhy->priv;

	#if DP83TG721_ONLY_TIMESTAMP_PTP_EVENTS
	if ((priv == NULL) || (msgType >= 8))
    {
        status = ENETPHY_EINVALIDPARAMS;
    }
	#else
    /* Only accept the PTP event message (msgType < 8) */
    if (priv == NULL)
    {
        status = ENETPHY_EINVALIDPARAMS;
    }
	#endif

    if (status == ENETPHY_SOK)
    {
        ListNode *node;
        TxTsWait *wait;
        bool existed = false;

        /* Check if it is existed or not */
        LIST_FOREACH(&priv->txTsWaitList, node)
        {
            wait = (TxTsWait *)node;
            if ((wait->msgType == msgType) && (wait->seqId == seqId))
            {
                existed = true;
                break;
            }
        }

        if (existed == false)
        {
            wait = (TxTsWait *)ListPop(&priv->txTsPoolList);
            if (wait != NULL)
            {
                wait->msgType = msgType;
                wait->seqId = seqId;
                wait->timeout = priv->tick + WAIT_TXTS_TIMEOUT;
                ListPush(&priv->txTsWaitList, &wait->node);
            }
            else
            {
                status = CSL_EALLOC-6;
                ENETTRACE_ERR("PHY %u: No slot to wait\n", hPhy->addr);
            }
        }
    }

    return status;
}

static int32_t Dp83tg721_getStatusFrameEthHeader(EnetPhy_Handle hPhy,
                    uint8_t *ethhdr, uint32_t size)
{
    int32_t status = ENETPHY_SOK;
    Dp83tg721Priv *priv = hPhy->priv;

    if ((ethhdr == NULL) || (size < sizeof(priv->stsFrameEthHdr)) || (priv == NULL))
    {
        status = ENETPHY_EBADARGS;
    }
    else
    {
        memcpy(ethhdr, priv->stsFrameEthHdr, sizeof(priv->stsFrameEthHdr));
    }

    return status;
}

static int32_t Dp83tg721_tickDriver(EnetPhy_Handle hPhy)
{
    Dp83tg721Priv *priv = hPhy->priv;
    int32_t status = ENETPHY_SOK;

    if (priv == NULL)
    {
        status = ENETPHY_EBADARGS;
    }
    else
    {
        priv->tick++;
    }

    return status;
}

static int32_t Dp83tg721_enableEventCapture(EnetPhy_Handle hPhy, uint32_t eventIdx,
                    bool falling, bool on)
{
    int32_t status = ENETPHY_SOK;

    if (eventIdx >= NUM_EVENTS)
    {
        status = ENETPHY_EINVALIDPARAMS;
    }

    if (status == ENETPHY_SOK)
    {
        uint16_t gpioSelect;
        uint16_t evnt = EVNT_WR | ENCODE_BIGFIELD(EVNT_SEL, eventIdx);
        uint8_t gpioNum = gGpioTable[eventIdx + START_EVENT_IDX];

        if (on == true)
        {
            evnt |= ENCODE_BIGFIELD(EVNT_GPIO, gpioNum);
            /*
            * The current state of the GPIO pin may cause an immediate timestamp
            * capture if the enables are set at the same time as the GPIO
            * selection field.
            * For example, if GPIO3 currently is currently at a logic high state when
            * the event monitor is set to that GPIO, the event monitor will see a
            * rising edge. To avoid this, software may program the GPIO selection
            * prior to setting the EVNT_RISE enable.
            */
            Dp83tg721_writeExtReg(hPhy, PTP_EVNT, evnt);

            if (falling)
            {
                evnt |= EVNT_FALL;
            }
            else
            {
                evnt |= EVNT_RISE;
            }

            /* PTP_EVENT_GPIO_SEL has multi-hot field to select multiple GPIO */
            gpioSelect = ExtRead(hPhy, PTP_EVENT_GPIO_SEL);
            gpioSelect |= ENCODE_BIGFIELD(PTP_GPIO_EVENT_EN, 1 << (gpioNum - 1));

            Dp83tg721_writeExtReg(hPhy, PTP_EVENT_GPIO_SEL, gpioSelect);
            Dp83tg721_writeExtReg(hPhy, PTP_EVNT, evnt);
        }
        else
        {
            gpioSelect = ExtRead(hPhy, PTP_EVENT_GPIO_SEL);
            gpioSelect &= ~ENCODE_BIGFIELD(PTP_GPIO_EVENT_EN, 1 << (gpioNum - 1));
            Dp83tg721_writeExtReg(hPhy, PTP_EVENT_GPIO_SEL, gpioSelect);
            Dp83tg721_writeExtReg(hPhy, PTP_EVNT, evnt);
        }
    }

    return status;
}

static int32_t Dp83tg721_enableTriggerOutput(EnetPhy_Handle hPhy, uint32_t triggerIdx,
                    uint64_t start, uint64_t period, bool repeat)
{
    int32_t status = ENETPHY_SOK;

    if (triggerIdx >= NUM_EVENTS)
    {
        status = ENETPHY_EINVALIDPARAMS;
    }

    if (status == ENETPHY_SOK)
    {
        uint16_t ptpTrig;
        uint16_t val;
        bool on = true;
        uint8_t gpioNum = gGpioTable[triggerIdx + START_TRIGGER_IDX];

        if (period == 0)
        {
            on = false;
        }

        ptpTrig = TRIG_WR | ENCODE_BIGFIELD(TRIG_CSEL, triggerIdx) |
            ENCODE_BIGFIELD(TRIG_GPIO, gpioNum) | TRIG_PULSE;

        if (repeat == true)
        {
            ptpTrig |= TRIG_PER;
        }

        val = ENCODE_BIGFIELD(TRIG_SEL, triggerIdx);

        if (on == false)
        {
            val |= TRIG_DIS;
            Dp83tg721_writeExtReg(hPhy, PTP_TRIG, ptpTrig);
            Dp83tg721_writeExtReg(hPhy, PTP_CTL, val);
        }
        else
        {
            uint32_t sec = start / SEC_NSEC;;
            uint32_t nsec = start % SEC_NSEC;
            uint32_t pwidth = period / 2;

            Dp83tg721_writeExtReg(hPhy, PTP_TRIG, ptpTrig);

            /* load trigger */
            val |= TRIG_LOAD;
            Dp83tg721_writeExtReg(hPhy, PTP_CTL, val);
            Dp83tg721_writeExtReg(hPhy, PTP_TDR, nsec & 0xffff); /* ns[15:0] */
            Dp83tg721_writeExtReg(hPhy, PTP_TDR, (nsec >> 16) & 0x3fff); /* ns[29:16] */
            Dp83tg721_writeExtReg(hPhy, PTP_TDR, sec & 0xffff);    /* sec[15:0] */
            Dp83tg721_writeExtReg(hPhy, PTP_TDR, sec >> 16);       /* sec[31:16] */
            Dp83tg721_writeExtReg(hPhy, PTP_TDR, pwidth & 0xffff); /* ns[15:0] */
            Dp83tg721_writeExtReg(hPhy, PTP_TDR, pwidth >> 16);    /* ns[31:16] */

            /* Triggers 0 and 1 has programmable pulsewidth2 */
            if (triggerIdx < 2) {
                Dp83tg721_writeExtReg(hPhy, PTP_TDR, pwidth & 0xffff);
                Dp83tg721_writeExtReg(hPhy, PTP_TDR, pwidth >> 16);
            }

            /* enable trigger */
            val &= ~TRIG_LOAD;
            val |= TRIG_EN;
            Dp83tg721_writeExtReg(hPhy, PTP_CTL, val);
        }
    }

    return status;
}

static int32_t Dp83tg721_getEventTs(EnetPhy_Handle hPhy, uint32_t *eventIdx,
                    uint32_t *seqId, uint64_t *ts64)
{
    int32_t status = ENETPHY_SOK;
    Dp83tg721Priv *priv = hPhy->priv;

    if ((eventIdx == NULL) || (seqId == NULL) || (ts64 == NULL) || (priv == NULL))
    {
        status = ENETPHY_EBADARGS;
    }
    else
    {
        EventTsReadyQ *readyQ = &priv->evTsReadyQ;
        EventTsInfo *event = &readyQ->eventInfoTable[readyQ->readP];
        if (event->ts != 0)
        {
            *eventIdx = event->index;
            *seqId = event->seqId;
            *ts64 = event->ts;
            readyQ->readP++;
            if (readyQ->readP == readyQ->size)
            {
                readyQ->readP = 0;
            }
            memset(event, 0, sizeof(EventTsInfo));
        }
        else
        {
            status = CSL_EALLOC-6;
        }
    }

    return status;
}
