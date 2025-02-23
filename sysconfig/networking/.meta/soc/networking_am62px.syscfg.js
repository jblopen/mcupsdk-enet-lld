
let common = system.getScript("/common");

const topModules_main = [
    "/networking/enet_cpsw/enet_cpsw",
];

const topModules_mcu = [

];

const topModules_a53 = [
    "/networking/enet_cpsw/enet_cpsw",
];

const driverVer = {
    "enet_cpsw": {
        version: "am62dx_am62ax_am62px",
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
