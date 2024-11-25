let common = system.getScript("/common");
let device = common.getDeviceName();

const cpswPhyExtendedConfig = new Map([
	 
	['DP83869',{extConfig:
`/* Extended PHY configuration for DP83869 */
.txClkShiftEn         = true,
.rxClkShiftEn         = true,
.txDelayInPs          = 2000U,   /* Value in pecosec. Refer to DLL_RX_DELAY_CTRL_SL field in ANA_RGMII_DLL_CTRL register of DP83869 PHY datasheet */
.rxDelayInPs          = 2000U,   /* Value in pecosec. Refer to DLL_TX_DELAY_CTRL_SL field in ANA_RGMII_DLL_CTRL register of DP83869 PHY datasheet */
.txFifoDepth          = 4U,
.impedanceInMilliOhms = 35000,  /* 35 ohms */
.idleCntThresh        = 4U,     /* Improves short cable performance */
.gpio0Mode            = DP83869_GPIO0_LED3,
.gpio1Mode            = DP83869_GPIO1_COL, /* Unused */
.ledMode              =
{
	DP83869_LED_LINKED,         /* Unused */
	DP83869_LED_LINKED_100BTX,
	DP83869_LED_RXTXACT,
	DP83869_LED_LINKED_1000BT,
},`}],

	['DP83867',{extConfig:
`.txClkShiftEn         = false,
.rxClkShiftEn         = false,
.txDelayInPs          = 2000U,  /* 2.00 ns */
.rxDelayInPs          = 2000U,  /* 2.00 ns */
.txFifoDepth          = 4U,     /* 4 bytes/nibbles */
.impedanceInMilliOhms = 50000U, /* 50 ohms */
.idleCntThresh        = 5U,
.gpio0Mode            = DP83867_GPIO0_RXERR,
.gpio1Mode            = DP83867_GPIO1_COL,
.ledMode			  =
{
	DP83867_LED_LINKED,
	DP83867_LED_LINKED_1000BT,
	DP83867_LED_RXTXACT,
	DP83867_LED_LINKED_100BTX,
},`}],

	['DP83822',{extConfig:""}],

	['DP83826',{extConfig:""}],

	['DP83TC812',{extConfig:""}],

	['DP83TG720',{extConfig:
`.txClkShiftEn = true,
.rxClkShiftEn = true,
.interruptEn = false,
.sgmiiAutoNegEn = true,
.MasterSlaveMode = DP83TG720_MASTER_SLAVE_STRAP,`}],

	['DP83TG721',{extConfig:""}],

	['CUSTOM',{extConfig:""}],

	['NO-PHY',{extConfig:""}],
],
);

function getPhyInfo(boardType)
{
    const cpswPhyInfoMap = 
		[
			{deviceName: "am261x-lp (dp83tg720 phy)", defaultPhy1: "DP83TG720", defaultPhyAddr1: 8, defaultPhy2: "DP83TG720", defaultPhyAddr2: 12},
			{deviceName: "am261x-lp (dp83826 phy)", defaultPhy1: "DP83826", defaultPhyAddr1: 3, defaultPhy2: "DP83826", defaultPhyAddr2: 1},
		];

		return cpswPhyInfoMap.find(element => element.deviceName === boardType);
}

function getConfigArr()
{
    return ethphy_devices;
}

function getExternalConfig(phyUsed, peripheral)
{
	return cpswPhyExtendedConfig.get(phyUsed).extConfig;
}

function getMaxInstanceCount()
{
	return 2; 
}

exports = {
    getConfigArr,
	getExternalConfig,
	getPhyInfo,
	getMaxInstanceCount,
};
