"use strict";

let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let device = common.getDeviceName();
let soc = system.getScript(`/networking/soc/networking_${common.getSocName()}`);
//Get packet pool configuration script
const pktPoolScript = system.getScript("./enet_pkt_pool_config");
//Get ALE configuration script
const aleScript = system.getScript("./enet_cpsw_ale_config");
//Get MDIO configuration script
const mdioScript = system.getScript("./enet_cpsw_mdio_config");
//Get Host Port configuration script
const hostportScript = system.getScript("./enet_cpsw_hostport_config");
//Get CPTS configuration script
const cptsScript = system.getScript("./enet_cpsw_cpts_config");
//Get MAC Port configuration script
const macportScript = system.getScript("./enet_cpsw_macport_config");
const utilsScript = system.getScript("./../../common/enet_cpsw_utils");
const pinMuxScript = system.getScript("./enet_cpsw_am64x_am243x_pinmux");

const enet_cpsw_pinmux_config = {
    name: "pinmuxConfig",
    displayName: "Pinmux config",
	longDescription: "Configuration of pinmux for CPSW",
    collapsed:true,
    config: [

    ],
};

const enet_cpsw_udma_channel_config = {
    name: "udmaChConfig",
    displayName: "DMA channel config",
	longDescription: "Configuration of Tx/Rx DMA channels",
    collapsed:true,
    config: [

    ],
};


const enet_cpsw_lwipIf_config = {
    name: "lwipIfConfig",
    displayName: "LWIP Interface config",
	longDescription: "Configuration of LWIP Interface",
    collapsed:true,
    config: [

    ],

};

const enet_cpsw_system_config = {
    name: "cpswSystemConfig",
    displayName: "System integration config",
    longDescription: "System integration related configuration",
    collapsed:true,
    config: [
        {
            name: "McmEnable",
            description: "Flag to enable multi-client manager. Required for multi-core, multiple Enet client use cases",
            displayName: "Mcm Enable",
            default: false,
        },
        {
            name: "ExternalPhyMgmtEnable",
            description: "Flag to enable phy management in application. The enet driver internal phy functions including phy state machine is bypassed in this mode",
            displayName: "External Phy Management Enable",
            default: false,
            onChange:function (inst, ui) {
                if(inst.ExternalPhyMgmtEnable == true) {
                    ui.macport1LinkSpeed.hidden = true;
                    ui.macport1LinkDuplexity.hidden = true;
                    ui.macport2LinkSpeed.hidden = true;
                    ui.macport2LinkDuplexity.hidden = true;
                }
                else{
                    ui.macport1LinkSpeed.hidden = false;
                    ui.macport1LinkDuplexity.hidden = false;
                    ui.macport2LinkSpeed.hidden = false;
                    ui.macport2LinkDuplexity.hidden = false;
                }
            }
        },
        {
            name: "RtosVariant",
            description: "Select FreeRTOS or No RTOS",
            displayName: "RTOS Variant",
            default: "FreeRTOS",
            options: [
                {
                    name: "FreeRTOS",
                    displayName: "FreeRTOS",
                },
                {
                    name: "NoRTOS",
                    displayName: "No RTOS (Bare Metal)",
                },
            ],
        },
        {
            name: "macAddrConfig",
            description: "MAC address to set in the driver. 'Auto Assign shall select the address automatiically from EEPROM and/or EFUSES. 'Manual Entry' will allow to input MAC address",
            displayName: "MAC Address Assignment Method",
            onChange:function (inst, ui) {
                if(inst.macAddrConfig === "Auto Assign") {
                    ui.macAddrList.hidden = true;
                } else {
                    ui.macAddrList.hidden = false;
                }
            },
            options: [
                {
                    name: "Auto Assign",
                },
                {
                    name: "Manual Entry",
                },
            ],
            default: "Auto Assign",
        },
        {
            name: "macAddrList",
            description: "MAC address to set in the driver. Enter MAC address. Seperate multiple MAC address with comma. Eg.: aa:bb:bb:cc:dd:ee,01:22:33:aa:bb:ee",
            displayName: "MAC Address List",
            default: "70:ff:76:1d:ec:f2,70:ff:76:1d:ec:e3",
            hidden: true,
        },
        {
            name: "AppLinkUpPortMask",
            description: "Application config to determine which macPorts should be polled for linkup to indicate link is up.Applicable in multi port scenario only",
            displayName: "AppLinkUpPortMask Config",
            default: "ANY_PORT",
            options: [
                {
                    name: "ALL_PORTS",
                },
                {
                    name: "ANY_PORT",
                },
            ],
        },
    ],
};

const enet_cpsw_board_config = {
    name: "cpswBoardConfig",
    displayName: "Board Config",
    longDescription: "Board specific configuration",
    collapsed:true,
    config: [
        {
            name: "customBoardEnable",
            description: "Enable Custom Board Configuration",
            displayName: "Custom Board",
            longDescription: "Configuration for custom board that are not supported out of box in MCU+ SDK",
            default: false,
        },
        {
            name: "disableOnboardMAC",
            description: "Use User defined MAC address for CPSW",
            displayName: "Disable on-board MAC Addresses",
            default: false,
            hidden: true,
        },
        {
            name: "useExtMACAddr",
            description: "Use User defined MAC address for CPSW",
            displayName: "Use User MAC Addr",
            default: false,
            hidden: true,
        },
    ],
};

function getInterfaceNameList(inst) {
    return pinMuxScript.getInterfaceNameList(inst);
}

function pinmuxRequirements(inst) {
    return pinMuxScript.pinmuxRequirements(inst);
}

function getPeripheralPinNames(inst)
{
    return pinMuxScript.getPeripheralPinNames(inst);
}

const enet_clock_config =
    {
        clockIds        : [ "TISCI_DEV_CPSW0" ],
        clockFrequencies: [
            {
                moduleId: "TISCI_DEV_CPSW0",
                clkId   : "TISCI_DEV_CPSW0_CPTS_RFT_CLK",
                clkRate : 200000000,
            },
        ],
    };

function getClockEnableIds(instance) {
    let instConfig = enet_clock_config;
    return instConfig.clockIds;
}

function getClockFrequencies(inst) {

    let instConfig = enet_clock_config;
    return instConfig.clockFrequencies;
}

function getDmaInterface(instance) {
    let cpswInstInfo = getCpswInstInfo(instance);
    return cpswInstInfo.dmaIf;
}

function getInstIdTable(instances) {
    let tbl = '{ '
    for (var i = 0; i < instances.length; i++)
    {
        tbl += '{';
        var matchedInst = getCpswInstInfo(instances[i])
        tbl += i + ', ' + matchedInst.enetType + ', ' +  matchedInst.instId
        tbl += '}, '
    }
    tbl += '}'
    return tbl;
}

function getMiiConfig(instance) {
    const cpswMiiConfigMap = new Map(
    [
        ["RGMII",{layerType:"ENET_MAC_LAYER_GMII", variantType:"ENET_MAC_VARIANT_FORCED", sublayerType:"ENET_MAC_SUBLAYER_REDUCED"}],
        ["RMII", {layerType:"ENET_MAC_LAYER_MII", variantType:"ENET_MAC_VARIANT_NONE", sublayerType:"ENET_MAC_SUBLAYER_REDUCED"}],
    ],)
    return cpswMiiConfigMap.get(instance.phyToMacInterfaceMode);
}

function getCpswInstInfo(instance) {
    const cpswInstInfoMap = new Map(
                               [
                                 ['awr294x',{enetType: 'ENET_CPSW_2G', numMacPorts: '1', instId: '0', dmaIf:'ENET_SOC_HOSTPORT_DMA_TYPE_CPDMA', macPortList:['ENET_MAC_PORT_1']}],
                                 ['am273x', {enetType: 'ENET_CPSW_2G', numMacPorts: '1', instId: '0', dmaIf:'ENET_SOC_HOSTPORT_DMA_TYPE_CPDMA', macPortList:['ENET_MAC_PORT_1']}],
                                 ['am263x',{enetType: 'ENET_CPSW_3G', numMacPorts: '2', instId: '0', dmaIf:'ENET_SOC_HOSTPORT_DMA_TYPE_CPDMA', macPortList:['ENET_MAC_PORT_1', 'ENET_MAC_PORT_2']}],
                                 ['am243x',{enetType: 'ENET_CPSW_3G', numMacPorts: '2', instId: '0', dmaIf:'ENET_SOC_HOSTPORT_DMA_TYPE_UDMA', macPortList:['ENET_MAC_PORT_1', 'ENET_MAC_PORT_2']}],
                                 ['am64x',{enetType: 'ENET_CPSW_3G', numMacPorts: '2', instId: '0', dmaIf:'ENET_SOC_HOSTPORT_DMA_TYPE_UDMA', macPortList:['ENET_MAC_PORT_1', 'ENET_MAC_PORT_2']}],
                               ],
                             );
    let instInfo =  cpswInstInfoMap.get(common.getSocName());
    instInfo.macPortList = instInfo.macPortList.filter(function(macPort, index,arr){
        let includeEntry = true;
        if ((macPort === 'ENET_MAC_PORT_1') && (instance.DisableMacPort1 === true))
        {
            includeEntry = false;
        }
        if ((macPort === 'ENET_MAC_PORT_2') && (instance.DisableMacPort2 === true))
        {
            includeEntry = false;
        }
        return includeEntry;
    });
    instInfo.numMacPorts = instInfo.macPortList.length;
    return instInfo;
}

function getBoardConfigTemplateInfo() {
    const boardConfigTemplate = new Map(
                               [
                                 ['am64x',{Cfile: "/board/ethphy_cpsw_icssg/templates/am64x_am243x/ethphy_cpsw_board_cfg.c.xdt",
								  Header: "/board/ethphy_cpsw_icssg/templates/am64x_am243x/ethphy_cpsw_board_cfg.h.xdt",}],
                                 ['am243x',{Cfile: "/board/ethphy_cpsw_icssg/templates/am64x_am243x/ethphy_cpsw_board_cfg.c.xdt",
                                  Header: "/board/ethphy_cpsw_icssg/templates/am64x_am243x/ethphy_cpsw_board_cfg.h.xdt"}],
                                 ['awr294x',{Cfile: "/board/ethphy_cpsw_icssg/templates/awr294x/ethphy_cpsw_board_cfg.c.xdt",
                                  Header: "/board/ethphy_cpsw_icssg/templates/awr294x/ethphy_cpsw_board_cfg.h.xdt"}],
                                 ['am273x', {Cfile: "/board/ethphy_cpsw_icssg/templates/am273x/ethphy_cpsw_board_cfg.c.xdt",
                                 Header: "/board/ethphy_cpsw_icssg/templates/am273x/ethphy_cpsw_board_cfg.h.xdt"}],
                                 ['am263x',{Cfile: "/board/ethphy_cpsw_icssg/templates/am263x/ethphy_cpsw_board_cfg.c.xdt",
                                 Header: "/board/ethphy_cpsw_icssg/templates/am263x/ethphy_cpsw_board_cfg.h.xdt"}],
                               ],
                             );
	return boardConfigTemplate.get(common.getSocName());
}


function getSocConfigTemplateInfo() {
    const socConfigTemplate = new Map(
                               [
                                 ['am64x',{Cfile: "/networking/enet_cpsw/templates/am64x_am243x/enet_soc_cfg.c.xdt"}],
                                 ['am243x',{Cfile: "/networking/enet_cpsw/templates/am64x_am243x/enet_soc_cfg.c.xdt"}],
                                 ['awr294x',{Cfile: "/networking/enet_cpsw/templates/awr294x/enet_soc_cfg.c.xdt"}],
                                 ['am273x', {Cfile: "/networking/enet_cpsw/templates/am273x/enet_soc_cfg.c.xdt"}],
                                 ['am263x',{Cfile: "/networking/enet_cpsw/templates/am263x/enet_soc_cfg.c.xdt"}],
                               ],
                             );
    return socConfigTemplate.get(common.getSocName());
}

function getPacketsCount(instance, channelType) {
    let totalNumPackets = 0;
    let driverVer = soc.getDriverVer("enet_cpsw");
    let dma_ch_instances;
    let module_dma_ch;

    if (channelType === "TX")
    {
        dma_ch_instances = instance.txDmaChannel;
        module_dma_ch = system.modules[`/networking/enet_cpsw/${driverVer}/enet_cpsw_${driverVer}_tx_channel`];
    }
    else
    {
        dma_ch_instances = instance.rxDmaChannel;
        module_dma_ch = system.modules[`/networking/enet_cpsw/${driverVer}/enet_cpsw_${driverVer}_rx_channel`];
    }

    for(let ch = 0; ch < dma_ch_instances.length; ch++) {
        let ch_instance = dma_ch_instances[ch];
        let ch_config = module_dma_ch.getInstanceConfig(ch_instance);
        totalNumPackets += ch_config.PacketsCount;
    }
    return totalNumPackets;
}

function getMacAddrCount(instance) {
    let totalMacAddr = 0;
    let driverVer = soc.getDriverVer("enet_cpsw");
    let dma_ch_instances = instance.rxDmaChannel;
    let module_dma_ch = system.modules[`/networking/enet_cpsw/${driverVer}/enet_cpsw_${driverVer}_rx_channel`];

    for(let ch = 0; ch < dma_ch_instances.length; ch++) {
        let ch_instance = dma_ch_instances[ch];
        let ch_config = module_dma_ch.getInstanceConfig(ch_instance);
        totalMacAddr += ch_config.macAddrCount;
    }
    return totalMacAddr;
}

function getChannelCount(instance, channelType) {
    let driverVer = soc.getDriverVer("enet_cpsw");
    let dma_ch_instances;
    let module_dma_ch;

    if (channelType === "TX")
    {
        dma_ch_instances = instance.txDmaChannel;
        module_dma_ch = system.modules[`/networking/enet_cpsw/${driverVer}/enet_cpsw_${driverVer}_tx_channel`];
    }
    else
    {
        dma_ch_instances = instance.rxDmaChannel;
        module_dma_ch = system.modules[`/networking/enet_cpsw/${driverVer}/enet_cpsw_${driverVer}_rx_channel`];
    }

    return dma_ch_instances.length;
}

function getTxPacketsCount(instance) {
    return getPacketsCount(instance, "TX");
}

function getRxPacketsCount(instance) {
    return getPacketsCount(instance, "RX");
}

function getTxChannelCount(instance) {
    return getChannelCount(instance, "TX");
}

function getRxChannelCount(instance) {
    return getChannelCount(instance, "RX");
}

function getChannelConfig(instance, channelType, chTypeInstNum) {
    let driverVer = soc.getDriverVer("enet_cpsw");
    let dma_ch_instances;
    let module_dma_ch;

    if (channelType === "TX")
    {
        dma_ch_instances = instance.txDmaChannel;
        module_dma_ch = system.modules[`/networking/enet_cpsw/${driverVer}/enet_cpsw_${driverVer}_tx_channel`];
    }
    else
    {
        dma_ch_instances = instance.rxDmaChannel;
        module_dma_ch = system.modules[`/networking/enet_cpsw/${driverVer}/enet_cpsw_${driverVer}_rx_channel`];
    }
    let channelCfgArray = new Array();


    for(let ch = 0; ch < dma_ch_instances.length; ch++) {
        let ch_instance = dma_ch_instances[ch];
        let ch_config = module_dma_ch.getInstanceConfig(ch_instance);
        channelCfgArray.push(ch_config);
    }
    return channelCfgArray[chTypeInstNum];
}

function getNetifCount(instance) {
    let instances = instance.netifInstance;
    return instances.length;
}

function getNetifConfig(instance, InstNum) {
    let driverVer = soc.getDriverVer("enet_cpsw");
    let instances;
    let module;

    instances = instance.netifInstance;
    module = system.modules[`/networking/enet_cpsw/${driverVer}/enet_cpsw_lwipif_netif`];

    let cfgArray = new Array();


    for(let num = 0; num < instances.length; num++) {
        let num_instance = instances[num];
        let num_config = module.getInstanceConfig(num_instance)[`moduleInstance`];
        cfgArray.push(num_config);
    }
    return cfgArray[InstNum];
}

function getDefaultNetifCount(instance)
{
    let defaultNetifCount = 0;

    for (let Idx = 0; Idx < getNetifCount(instance); Idx++)
    {
        defaultNetifCount += (getNetifConfig(instance, Idx).isDefault === true) ? 1 : 0;
    }
    return defaultNetifCount;

}

function getDefaultNetifIdx(instance)
{
    let defaultNetifIdx = -1;

    for (let Idx = 0; Idx < getNetifCount(instance); Idx++)
    {
        if(getNetifConfig(instance, Idx).isDefault === true)
        {
            defaultNetifIdx = Idx;
            break;
        }
    }
    return defaultNetifIdx;
}

function getDefaultPacketCount(channelType) {
    let driverVer = soc.getDriverVer("enet_cpsw");
    let module_dma_ch;

    if (channelType === "TX")
    {
        module_dma_ch = system.modules[`/networking/enet_cpsw/${driverVer}/enet_cpsw_${driverVer}_tx_channel`];
    }
    else
    {
        module_dma_ch = system.modules[`/networking/enet_cpsw/${driverVer}/enet_cpsw_${driverVer}_rx_channel`];
    }
    return (module_dma_ch.config.filter(o => o.name === 'PacketsCount'))[0].default;
}


function getCpuID() {
    return system.getScript(`/drivers/soc/drivers_${common.getSocName()}`).getCpuID();
}


function getRxDefaultFlowCount(instance)
{
    let defaultFlowCount = 0;

    for (let chIdx = 0; chIdx < getRxChannelCount(instance); chIdx++)
    {
        defaultFlowCount += (getChannelConfig(instance, "RX", chIdx).useDefaultFlow === true) ? 1 : 0;
    }
    return defaultFlowCount;


}

function validate(instance, report) {
    pktPoolScript.validate(instance, report);
    aleScript.validate(instance, report);
    mdioScript.validate(instance, report);
    macportScript.validate(instance, report);
    hostportScript.validate(instance, report);
    if (getRxDefaultFlowCount(instance) != 1)
    {
        report.logError(`One rx  and only one flow should be designated as default rx flow`, instance, "rxDmaChannel");

    }

    let numNetifsCount = getNetifCount(instance);
    if (numNetifsCount > 0)
    {
        if (getDefaultNetifCount(instance) != 1)
        {
            report.logError(`Only one netif can be set as default`, instance, "netifInstance");
        }

        if (numNetifsCount === 2)
        {
            if ((instance.DisableMacPort1 === true) || (instance.DisableMacPort2 === true))
            {
                report.logError("Both MAC ports in 'MAC PORT Config' should be enabled to support two NetIfs", instance);
            }

            if ((instance.macOnlyEn_hostPort === false) || (instance.macOnlyEn_macPort1 === false) || (instance.macOnlyEn_macPort2 === false))
            {
                report.logError("All Ports in 'ALE Config -> ALE Port Config -> MAC-only mode config' should be in MAC-only mode in case of two NetIfs", instance);
            }
        }

        if (getMacAddrCount(instance) < numNetifsCount)
        {
             report.logError("Number of MAC address allocated is not enough to number of LwIP NetIFs", instance);
        }
    }

    if (/^([0-9a-fA-F]{2}[:-]){5}[0-9a-fA-F]{2}(,([0-9a-fA-F]{2}[:-]){5}[0-9a-fA-F]{2})+/.test(instance.macAddrList) == false)
    {
        report.logError(`Invalid macAddrList Entry`, instance, "macAddrList");
    }
}

function moduleInstances(instance) {

    let Instances = new Array();
    let driverVer = soc.getDriverVer("enet_cpsw");
    let maxCh     = 8;
    let maxNetif  = 2;

    Instances.push({
        name: "txDmaChannel",
        displayName: "ENET tx dma channel",
        moduleName: `/networking/enet_cpsw/${driverVer}/enet_cpsw_${driverVer}_tx_channel`,
        useArray: true,
        minInstanceCount: 1,
        maxInstanceCount: maxCh,
        defaultInstanceCount: 1,
        collapsed:false,
        group: "udmaChConfig",
    });

    Instances.push({
        name: "rxDmaChannel",
        displayName: "ENET rx dma channel",
        moduleName: `/networking/enet_cpsw/${driverVer}/enet_cpsw_${driverVer}_rx_channel`,
        useArray: true,
        minInstanceCount: 1,
        maxInstanceCount: maxCh,
        defaultInstanceCount: 1,
        collapsed:false,
        group: "udmaChConfig",
    });

    Instances.push({
        name: "netifInstance",
        displayName: "NETIF instance",
        moduleName: `/networking/enet_cpsw/${driverVer}/enet_cpsw_lwipif_netif`,
        useArray: true,
        minInstanceCount: 0,
        maxInstanceCount: maxNetif,
        defaultInstanceCount: 0,
        collapsed:false,
        group: "lwipIfConfig",
    });

    return (Instances);
}

function addSharedModuleInstances(inst) {
    let modInstances = new Array();

    modInstances.push({
        name: "udmaDrv",
        displayName: "Udma",
        moduleName: "/drivers/udma/udma",
        requiredArgs: {
            instance: "PKTDMA_0",
            $name: "CONFIG_UDMA_PKTDMA_0",
        },
	});

    if((inst.DisableMacPort1 === false) && (inst.customBoardEnable === false)){
        modInstances.push({
            name: "ethphy1",
            displayName: "Port 1 PHY Configuration",
            moduleName: "/board/ethphy_cpsw_icssg/ethphy_cpsw_icssg",
            requiredArgs: {
                peripheral: "CPSW_MAC_PORT_1",
                enableCustomBoard: inst.customBoardEnable,
            },
            group: "macPort1Cfg",
            });
    }

    if((inst.DisableMacPort2 == false) && (inst.customBoardEnable === false)){
        modInstances.push({
            name: "ethphy2",
            displayName: "Port 2 PHY Configuration",
            moduleName: "/board/ethphy_cpsw_icssg/ethphy_cpsw_icssg",
            requiredArgs: {
                peripheral: "CPSW_MAC_PORT_2",
                enableCustomBoard: inst.customBoardEnable,
            },
            group: "macPort2Cfg",
            });
    }

    return modInstances;
}

function getCpuInfo() {
	const cpuInfo = new Map(
                               [
                                 ['CSL_CORE_ID_R5FSS0_0',{subsystem: "R5FSS",
                                  clusternum: "0", core: "0"}],
                                 ['CSL_CORE_ID_R5FSS0_1',{subsystem: "R5FSS",
                                  clusternum: "0", core: "1"}],
                                 ['CSL_CORE_ID_R5FSS1_0',{subsystem: "R5FSS",
                                  clusternum: "1", core: "0"}],
                                 ['CSL_CORE_ID_R5FSS1_1', {subsystem: "R5FSS",
                                  clusternum: "1", core: "1"}],
                                 ['CSL_CORE_ID_A53SS0_0',{subsystem: "A53SS",
                                  clusternum: "0", core: "0"}],
                               ],
                             );
	return cpuInfo.get(getCpuID());
}



function getEnetCoreIntNumPrefix() {
    const coreInfo = getCpuInfo();
    if(common.getSelfSysCfgCoreName().includes("a53")) {
        return  "CSLR_GICSS0_SPI_"
    }

    if ((coreInfo) && (common.getSelfSysCfgCoreName().includes("r5f"))) {
        return `CSLR_${coreInfo.subsystem}${coreInfo.clusternum}_CORE${coreInfo.core}_INTR_`
    }
}

function getEnetCoreIdPrefix() {
    let coreInfo = getCpuInfo();

    if(common.getSelfSysCfgCoreName().includes("a53")) {
        return `TISCI_DEV_${coreInfo.subsystem}${coreInfo.clusternum}_CORE_${coreInfo.core}`;
    }

    
    if((coreInfo) && (common.getSelfSysCfgCoreName().includes("r5f"))) {
        return `TISCI_DEV_${coreInfo.subsystem}${coreInfo.clusternum}_CORE${coreInfo.core}`;
	}
}

let enet_cpsw_module_name = "/networking/enet_cpsw/enet_cpsw";

let enet_cpsw_module = {

    displayName: "Enet (CPSW)",
    longDescription: "Driver for Common Port SWitch (CPSW). Support MAC, Switch and used in auto and industrial Ethernet to run TCP/IP, AVB etc. applications. TSN is supported via CPSW",
    templates: {
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: enet_cpsw_module_name,
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/networking/enet_cpsw/templates/enet_cpsw_v0.h.xdt",
            moduleName: enet_cpsw_module_name,
        },
        "/drivers/system/power_clock_config.c.xdt": {
            moduleName: enet_cpsw_module_name,
        },
        "/board/board/board_config.h.xdt": {
            board_config: getBoardConfigTemplateInfo().Header,
            moduleName: enet_cpsw_module_name,
        },
        "/board/board/board_config.c.xdt": {
            board_config: getBoardConfigTemplateInfo().Cfile,
            moduleName: enet_cpsw_module_name,
        },
        "/networking/common/enet_config.c.xdt": {
            enet_mem_config: "/networking/enet_cpsw/templates/enet_app_memutils_cfg_udma.c.xdt",
            enet_syscfg_info: "/networking/enet_cpsw/templates/enet_app_syscfg_info.c.xdt",
            moduleName: enet_cpsw_module_name,
        },
        "/networking/common/enet_config.h.xdt": {
            enet_config: "/networking/enet_cpsw/templates/enet_syscfg.h.xdt",
            moduleName: enet_cpsw_module_name,
        },
        "/networking/common/enet_open.c.xdt": {
            enet_open: "/networking/enet_cpsw/templates/enet_init.c.xdt",
            enet_init_config: "/networking/enet_cpsw/templates/enet_app_cpsw_cfg.c.xdt",
            moduleName: enet_cpsw_module_name,
        },
        "/networking/common/enet_open.h.xdt": {
            enet_open: "/networking/enet_cpsw/templates/enet_init.h.xdt",
            moduleName: enet_cpsw_module_name,
        },
        "/networking/common/enet_soc.c.xdt": {
            enet_soc: getSocConfigTemplateInfo().Cfile,
            moduleName: enet_cpsw_module_name,
        },
        "/networking/common/enet_lwipif.c.xdt": {
            enet_lwipif: "/networking/enet_cpsw/templates/enet_lwipif.c.xdt",
            moduleName: enet_cpsw_module_name,
        },
        "/networking/common/enet_lwipif.h.xdt": {
            enet_lwipif: "/networking/enet_cpsw/templates/enet_lwipif.h.xdt",
            moduleName: enet_cpsw_module_name,
        },
    },
    defaultInstanceName: "CONFIG_ENET_CPSW",
    config: [
        enet_cpsw_system_config,
        enet_cpsw_udma_channel_config,
        pktPoolScript.config,
        enet_cpsw_lwipIf_config,
        aleScript.config,
        mdioScript.config,
        hostportScript.config,
        macportScript.config,
        enet_cpsw_board_config,
        cptsScript.config,
        pinMuxScript.config,
    ],
    moduleStatic: {
        modules: function(inst) {
            return [{
                name: "system_common",
                moduleName: "/system_common",
            }]
        },
    },
    moduleInstances: moduleInstances,
    utils: utilsScript,
    sharedModuleInstances: addSharedModuleInstances,
    pinmuxRequirements,
    getInterfaceNameList,
    getPeripheralPinNames,
    getClockEnableIds,
    getClockFrequencies,
    getDmaInterface,
    getInstIdTable,
    getCpswInstInfo,
    getCpuID,
    getCpuInfo,
    getEnetCoreIntNumPrefix,
    getEnetCoreIdPrefix,
    getSocConfigTemplateInfo,
    getTxPacketsCount,
    getRxPacketsCount,
    getRxChannelCount,
    getTxChannelCount,
    getChannelConfig,
    getNetifCount,
    getNetifConfig,
    getDefaultNetifIdx,
    getDefaultPacketCount,
    getMiiConfig,
    validate: validate,
};

exports = enet_cpsw_module;