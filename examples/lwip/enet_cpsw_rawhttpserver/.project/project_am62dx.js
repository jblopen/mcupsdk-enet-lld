let path = require('path');

let device = "am62dx";

const files = {
    common: [
            "app_cpswconfighandler.c",
            "app_main.c",
            "main.c",
            "app_httpsever.c",
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

const libdirs_nortos = {
    common: [
        "generated",
        "${MCU_PLUS_SDK_PATH}/source/kernel/nortos/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/source/board/lib",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/lib",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lib",

    ],
};

const includes_nortos_r5f = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/board/ethphy/enet/rtos_drivers/include",
        "${MCU_PLUS_SDK_PATH}/source/board/ethphy/port",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/utils/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/utils/V3",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include/phy",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include/core",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/soc/k3/am62dx",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/hw_include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/hw_include/mdio/V4",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/src/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-port/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-port/nortos/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/lwipif/inc",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/contrib",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-config/am62dx",
    ],
};

const libs_nortos_r5f = {
    common: [
        "nortos.am62dx.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am62dx.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am62dx.r5f.ti-arm-clang.${ConfigName}.lib",
        "enet-cpsw.am62dx.r5f.ti-arm-clang.${ConfigName}.lib",
        "lwipif-cpsw-nortos.am62dx.r5f.ti-arm-clang.${ConfigName}.lib",
        "lwip-nortos.am62dx.r5f.ti-arm-clang.${ConfigName}.lib",
        "lwip-contrib-nortos.am62dx.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const linker_includePath_nortos = {
    common: [
        "${PROJECT_BUILD_DIR}/syscfg",

    ],
};

const defines_r5f = {
    common: [
        "SOC_AM62DX",
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

const readmeDoxygenPageTag = "EXAMPLES_ENET_LWIP_CPSW_HTTPSERVER";

const templates_nortos_r5f =
[
    {
        input: "source/networking/enet/core/sysconfig/.project/templates/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "appMain",
        },
    },
];


const buildOptionCombos = [
    { device: device, cpu: "mcu-r5fss0-0", cgt: "ti-arm-clang", board: "am62dx-evm", os: "nortos"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "enet_cpsw_rawhttpserver";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.lnkfiles = lnkfiles;
    build_property.syscfgfile = syscfgfile;
    build_property.projecspecFileAction = "link";
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;
    if(buildOption.cpu.match(/r5f*/)) {
        if(buildOption.os.match(/nortos*/) )
        {
            const _ = require('lodash');
            let libdirs_nortos_cpy = _.cloneDeep(libdirs_nortos);
            /* Logic to remove generated/ from libdirs_nortos, it generates warning for ccs build */
            if (buildOption.isProjectSpecBuild === true)
            {
                var delIndex = libdirs_nortos_cpy.common.indexOf('generated');
                if (delIndex !== -1) {
                    libdirs_nortos_cpy.common.splice(delIndex, 1);
                }
            }
            build_property.includes = includes_nortos_r5f;
            build_property.libdirs = libdirs_nortos_cpy;
            build_property.templates = templates_nortos_r5f;
            if(buildOption.cgt.match(/gcc*/))
            {
                build_property.libs = libs_nortos_r5f_gcc;
                build_property.lnkpreprocessor_gcc = lnkpreprocessor_gcc;
            }
            else
            {
                build_property.libs = libs_nortos_r5f;
                build_property.cflags = cflags_r5f;
                build_property.lflags = lflags_r5f;
            }
            build_property.defines = defines_r5f;
            build_property.projectspecLnkPath = linker_includePath_nortos;
        }
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
