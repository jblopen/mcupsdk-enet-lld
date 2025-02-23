
let common = system.getScript("/common");

const topModules_main = [
    "/networking/enet_icss/enet_icss",
    "/networking/enet_cpsw/enet_cpsw",
    "/networking/icss_emac/icss_emac",
];

const topModules_mcu = [

];

const topModules_a53 = [
    "/networking/enet_icss/enet_icss",
    "/networking/enet_cpsw/enet_cpsw",
    "/networking/icss_emac/icss_emac",

];

const driverVer = {
    "enet_cpsw": {
        version: "am64x_am243x",
    },
}

exports = {
    getTopModules: function() {

        let topModules = topModules_main;

        if(common.getSelfSysCfgCoreName().includes("m4f")) {
            topModules = topModules_mcu;
        }
        if (common.getSelfSysCfgCoreName().match(/a53*/))
        {
            topModules = topModules_a53;
        }

        return topModules;
    },
    getDriverVer: function(driverName) {
        return driverVer[driverName].version;
    },
    getDirName: function(driverName) {
        return driverVer[driverName].version;
    },
};