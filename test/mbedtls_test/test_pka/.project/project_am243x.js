let path = require('path');

let device = "am243x";

const files = {
    common: [
        "main.c",
        "mbedtls_main.c",
        "mbedtls_rsa.c",
        "mbedtls_ecdsa.c",
        "mbedtls_test_utils.c"
    ],
};

const filedirs = {
    common: [
        "..",
        "../..",
        "../../..",
        "../../../../utils",
    ],
};

const libdirs_freertos = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/source/board/lib",
        "${MCU_PLUS_SDK_PATH}/test/unity/lib",
        "${MCU_PLUS_SDK_PATH}/source/networking/mbedtls_library/lib",
        "${MCU_PLUS_SDK_PATH}/source/security/lib",
        "${MCU_PLUS_SDK_PATH}/source/kernel/dpl",
    ],
};

const includes_freertos_r5f = {
    common: [
        "generated",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am243x/r5f",
        "${MCU_PLUS_SDK_PATH}/source/networking/mbedtls_library/mbedtls/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/mbedtls_library/mbedtls_ti",
        "${MCU_PLUS_SDK_PATH}/test/unity/",
        "${MCU_PLUS_SDK_PATH}/source",
        "${MCU_PLUS_SDK_PATH}/source/security",
        "${MCU_PLUS_SDK_PATH}/source/security/crypto/sa2ul/",
        "${MCU_PLUS_SDK_PATH}/source/security/crypto/",
        "${MCU_PLUS_SDK_PATH}/source/kernel/dpl/",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/test/mbedtls_test/",
    ],
};

const libs_freertos_r5f = {
    common: [
        "freertos.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "mbedtls.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "unity.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "security.am243x.r5f.ti-arm-clang.${ConfigName}.lib"
    ],
};

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg"

const readmeDoxygenPageTag = "EXAMPLES_EMPTY";

const templates_freertos_r5f =
[
    {
        input: "source/networking/enet/core/sysconfig/.project/templates/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "mbedtls_test_main",
        },
    }
];

const defines_r5f = {
    common: [
        "MBEDTLS_CONFIG_FILE=\\\"alt_config.h\\\"",
    ],
};

const cflags = {
    release: [
        "-Oz",
    ],
};

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "freertos"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-lp", os: "freertos"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "test_mbedtls_pka";
    property.isInternal = true;
    property.tirexResourceSubClass = [ "example.gettingstarted" ];
    property.description = "This is the test suite for mbedTLS Entropy and ECDSA"
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.lnkfiles = lnkfiles;
    build_property.syscfgfile = syscfgfile;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;
    build_property.cflags = cflags;

    if(buildOption.cpu.match(/r5f*/)) {
        if(buildOption.os.match(/freertos*/) )
        {
            build_property.includes = includes_freertos_r5f;
            build_property.libdirs = libdirs_freertos;
            build_property.libs = libs_freertos_r5f;
            build_property.templates = templates_freertos_r5f;
            build_property.defines = defines_r5f;
        }
    }
    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
