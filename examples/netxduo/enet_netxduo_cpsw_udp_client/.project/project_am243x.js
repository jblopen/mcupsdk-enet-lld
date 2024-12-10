let path = require('path');

let device = "am243x";

const files = {
    common: [
        "app_udpclient.c",
        "app_cpswconfighandler.c",
        "main.c",
    ],
};

/* Relative to where the makefile will be generated
 * Typically at <example_folder>/<BOARD>/<core_os_combo>/<compiler>
 */
const filedirs = {
    common: [
        "..",       /* core_os_combo base */
        "../../..", /* Example base */
    ],
};


const libdirs = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/lib",
        "${MCU_PLUS_SDK_PATH}/source/kernel/threadx/lib",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/source/board/lib",
    ],
};

const includes = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/board/ethphy/enet/rtos_drivers/include",
        "${MCU_PLUS_SDK_PATH}/source/board/ethphy/port",
        "${MCU_PLUS_SDK_PATH}/source/kernel/threadx/threadx_src/common/inc",
        "${MCU_PLUS_SDK_PATH}/source/kernel/threadx/ports/ti_arm_gcc_clang_cortex_r5/inc",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/common/inc",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_enet",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/ports/cortex_r5/gnu/inc/",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/utils/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/utils/V3",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include/phy",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include/core",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/soc/k3/am64x_am243x",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/hw_include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/hw_include/mdio/V4",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/auto_ip",
        // "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/azure_iot",
        // "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/BSD",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/cloud",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/dhcp",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/dns",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/ftp",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/http",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/mdns",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/mqtt",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/nat",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/pop3",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/ppp",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/pppoe",
        // "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/ptp",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/rtp",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/rtsp",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/smtp",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/snmp",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/sntp",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/telnet",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/tftp",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/web",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/websocket",
    ],
};

const libs = {
    common: [
        "netxduo.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "threadx.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "enet-cpsw.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const libs_gcc = {
    common: [
        "netxduo.am243x.r5f.gcc-armv7.${ConfigName}.lib",
        "threadx.am243x.r5f.gcc-armv7.${ConfigName}.lib",
        "enet-cpsw.am243x.r5f.gcc-armv7.${ConfigName}.lib",
        "drivers.am243x.r5f.gcc-armv7.${ConfigName}.lib",
        "board.am243x.r5f.gcc-armv7.${ConfigName}.lib",
    ],
};

const defines_r5f = {
    common: [
        "NX_INCLUDE_USER_DEFINE_FILE",
        "ENET_ENABLE_PER_CPSW=1",
    ],
};

const cflags_r5f = {
    common: [
    ],
    release: [
        "-Oz",
        "-flto",
    ],
};

const lflags_r5f = {
    common: [
        "--zero_init=on",
        "--use_memset=fast",
        "--use_memcpy=fast"
    ],
};

const loptflags_r5f = {
    release: [
        "-mcpu=cortex-r5",
        "-mfloat-abi=hard",
        "-mfpu=vfpv3-d16",
        "-mthumb",
        "-Oz",
        "-flto"
    ],
};

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg";

const readmeDoxygenPageTag = "EXAMPLES_ECLIPSE_THREADX_NETXDUO_CPSW_UDP_CLIENT";

const templates_gcc =
[
    {
        input: ".project/templates/am243x/common/linker_r5f_gcc.cmd.xdt",
        output: "linker.cmd",
    },
    {
        input: ".project/templates/am243x/threadx/main_threadx.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "netxduo_cpsw_main",
        },
    }
];

const templates =
[
    {
        input: ".project/templates/am243x/threadx/main_threadx.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "netxduo_cpsw_main",
        },
    }
];

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "threadx"},
    { device: device, cpu: "r5fss0-0", cgt: "gcc-armv7",    board: "am243x-evm", os: "threadx"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-lp", os: "threadx"},
    { device: device, cpu: "r5fss0-0", cgt: "gcc-armv7",    board: "am243x-lp", os: "threadx"}
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "enet_netxduo_cpsw_udp_client";
    property.isInternal = false;
    property.tirexResourceSubClass = [ "example.gettingstarted" ];
    property.description = "A simple UDP client example for NetxDuo with CPSW."
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.libdirs = libdirs;
    build_property.lnkfiles = lnkfiles;
    build_property.syscfgfile = syscfgfile;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;
    build_property.projecspecFileAction = "link";
    build_property.includes = includes;
    build_property.templates = templates;
    if(buildOption.cgt.match(/gcc*/)) {
        build_property.libs = libs_gcc;
    } else {
        build_property.libs = libs;
        build_property.cflags = cflags_r5f;
        build_property.lflags = lflags_r5f;
        build_property.loptflags = loptflags_r5f;
    }
    build_property.defines = defines_r5f;

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
