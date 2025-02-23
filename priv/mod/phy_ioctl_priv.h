/*
 *  Copyright (c) Texas Instruments Incorporated 2020
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
 * \file  phy_priv.h
 *
 * \brief This file contains private type definitions and helper macros for the
 *        PHY module which are meant for internal use in Enet Per drivers.
 */

#ifndef PHY_IOCTL_PRIV_H_
#define PHY_IOCTL_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <include/core/enet_mod_phy.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */




/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */



/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_GET_ID(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms);
int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_GET_SUPPORTED_MODES(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms);
int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_GET_LOOPBACK_STATE(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms);
int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_IS_ALIVE(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms);
int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_IS_LINKED(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms);
int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_GET_LINK_MODE(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms);
int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_RESET(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms);
int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_READ_REG(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms);
int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_WRITE_REG(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms);
int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_READ_EXT_REG(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms);
int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_WRITE_EXT_REG(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms);
int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_C45_READ_REG(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms);
int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_C45_WRITE_REG(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms);
int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_PRINT_REGS(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms);
int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_ADJ_PTP_FREQ(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms);
int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_ADJ_PTP_PHASE(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms);
int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_GET_PTP_TIME(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms);
int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_SET_PTP_TIME(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms);
int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_GET_PTP_TXTS(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms);
int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_GET_PTP_RXTS(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms);
int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_WAIT_PTP_TXTS(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms);
int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_PROC_STATUS_FRAME(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms);
int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_GET_STATUS_FRAME_ETHDR(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms);
int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_ENABLE_PTP(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms);
int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_ENABLE_EVENT_CAPTURE(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms);
int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_ENABLE_TRIGGER_OUTPUT(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms);
int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_GET_EVENT_TIMESTAMP(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms);

/* ========================================================================== */
/*                        Deprecated Function Declarations                    */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* PHY_IOCTL_PRIV_H_ */
