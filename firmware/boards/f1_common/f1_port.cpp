#include "port.h"

#include "wideband_config.h"

#include "hal.h"
#include "hal_mfs.h"

/* communication with OpenBLT that is plain C, not to modify external file
 * Same code used to store "DFU-requested" flag */
extern "C" {
    #include "openblt/shared_params.h"
};

// Storage
// TODO: runtime detection?
static const MFSConfig mfscfg1 = {
    .flashp           = (BaseFlash *)&EFLD1,
    .erased           = 0xFFFFFFFFU,
#ifdef STM32F103xB
    /* 128K flash device with 1K pages
     * use last 8 pages for settings
     * one bank is 4K */
    .bank_size        = 4096U,
    .bank0_start      = 120U,
    .bank0_sectors    = 4U,
    .bank1_start      = 124U,
    .bank1_sectors    = 4U
#endif
#ifdef STM32F103xE
    /* 256K flash device with 2K pages
     * use last 8 pages for settings
     * one bank is 8K */
    .bank_size        = 8096U,
    .bank0_start      = 120U,
    .bank0_sectors    = 4U,
    .bank1_start      = 124U,
    .bank1_sectors    = 4U
#endif
};

static MFSDriver mfs1;
static mfs_nocache_buffer_t __nocache_mfsbuf;

// Settings
static Configuration cfg;
#define MFS_CONFIGURATION_RECORD_ID     1

#ifndef BOARD_DEFAULT_SENSOR_TYPE
#define BOARD_DEFAULT_SENSOR_TYPE SensorType::LSU49
#endif

// Configuration defaults
void Configuration::LoadDefaults()
{
    *this = {};

    NoLongerUsed0 = 0;
    sensorType = BOARD_DEFAULT_SENSOR_TYPE;

    /* default auxout curve is 0..5V for AFR 8.5 to 18.0
     * default auxout[n] input is AFR[n] */
    for (size_t i = 0; i < 8; i++) {
        auxOutBins[0][i] = auxOutBins[1][i] = 8.5 + (18.0 - 8.5) / 7 * i;
        auxOutValues[0][i] = auxOutValues[1][i] = 0.0 + (5.0 - 0.0) / 7 * i;
    }
    auxOutputSource[0] = AuxOutputMode::Afr0;
    auxOutputSource[1] = AuxOutputMode::Afr1;

    for (size_t i = 0; i < AFR_CHANNELS; i++) {
        // enable RusEFI protocol
        afr[i].RusEfiTx = true;
        afr[i].RusEfiTxDiag = true;
        afr[i].RusEfiIdOffset = 2 * i;

        // Disable AemNet
        afr[i].AemNetTx = false;
        afr[i].AemNetIdOffset = i;
    }

    for (size_t i = 0; i < EGT_CHANNELS; i++) {
        // disable RusEFI protocol - not implemented
        egt[i].RusEfiTx = false;
        egt[i].RusEfiTxDiag = false;
        egt[i].RusEfiIdOffset = i;

        // Enable AemNet
        egt[i].AemNetTx = true;
        egt[i].AemNetIdOffset = i;
    }

    iobox.idx = 0;
    iobox.enable_rx = 0;
    iobox.enable_tx = 0;
    iobox.IDE = CAN_IDE_STD;
    iobox.SID = 0x200;

    /* Finaly */
    Tag = ExpectedTag;
}

void Configuration::LoadDefaults(uint16_t option)
{
    LoadDefaults();

    // Override default with specific options
    switch (option) {
    case 0:
        //nop
        break;
    case 1:
        // AEM protocol AFR + EGT
        for (size_t i = 0; i < EGT_CHANNELS; i++) {
            egt[i].AemNetTx = true;
        }
        for (size_t i = 0; i < AFR_CHANNELS; i++) {
            afr[i].RusEfiTx = false;
            afr[i].RusEfiTxDiag = false;
            afr[i].AemNetTx = true;
        }
        break;
    case 2:
        // AEM AFR only
        for (size_t i = 0; i < EGT_CHANNELS; i++) {
            egt[i].AemNetTx = false;
        }
        for (size_t i = 0; i < AFR_CHANNELS; i++) {
            afr[i].RusEfiTx = false;
            afr[i].RusEfiTxDiag = false;
            afr[i].AemNetTx = true;
        }
        break;
    case 16:
    case 17:
    case 18:
        // DAC over can using MS IO Box protocol (rx only)
        auxOutputSource[0] = auxOutputSource[1] = AuxOutputMode::MsIoBox;
        for (size_t i = 0; i < EGT_CHANNELS; i++) {
            egt[i].AemNetTx = false;
        }
        for (size_t i = 0; i < AFR_CHANNELS; i++) {
            afr[i].RusEfiTx = false;
            afr[i].RusEfiTxDiag = false;
            afr[i].AemNetTx = false;
        }
        if (option - 16 < 3) {
            iobox.idx = option - 16;
        } else {
            // custom CAN ID
            iobox.idx = 3;
        }
        iobox.EID = 0x200 + 0x20 * (option - 16);
        iobox.enable_rx = 1;
        break;
    default:
        break;
    }
}

int InitConfiguration()
{
    size_t size = GetConfigurationSize();

    /* Starting EFL driver.*/
    eflStart(&EFLD1, NULL);

    mfsObjectInit(&mfs1, &__nocache_mfsbuf);

    mfs_error_t err = mfsStart(&mfs1, &mfscfg1);
    if (err != MFS_NO_ERROR) {
        return -1;
    }

    err = mfsReadRecord(&mfs1, MFS_CONFIGURATION_RECORD_ID, &size, GetConfigurationPtr());
    if ((err != MFS_NO_ERROR) || (size != GetConfigurationSize() || !cfg.IsValid())) {
        /* load defaults */
        cfg.LoadDefaults(0);
    }

    return 0;
}

Configuration* GetConfiguration()
{
    return &cfg;
}

void SetConfiguration()
{
    SaveConfiguration();
}

void ResetConfiguration(uint16_t option)
{
    cfg.LoadDefaults(option);
}

/* TS stuff */
int SaveConfiguration() {
    /* TODO: handle error */
    mfs_error_t err = mfsWriteRecord(&mfs1, MFS_CONFIGURATION_RECORD_ID, GetConfigurationSize(), GetConfigurationPtr());
    if (err != MFS_NO_ERROR) {
        return -1;
    }
    return 0;
}

uint8_t *GetConfigurationPtr()
{
    return (uint8_t *)&cfg;
}

size_t GetConfigurationSize()
{
    return sizeof(cfg);
}

const char *getTsSignature() {
    return TS_SIGNATURE;
}

SensorType GetSensorType()
{
    return cfg.sensorType;
}

void rebootNow()
{
    NVIC_SystemReset();
}

void rebootToOpenblt()
{
#ifdef USE_OPENBLT
    /* safe to call on already inited shares area */
    SharedParamsInit();
    /* Store flag to stay in OpenBLT */
    SharedParamsWriteByIndex(0, 0x01);

    rebootNow();
#endif
}

void rebootToDfu()
{
    /* safe to call on already inited shares area */
    SharedParamsInit();
    /* Store flag to jump to DFU at main FW init */
    SharedParamsWriteByIndex(0, 0x02);

    rebootNow();
}

// stm32f10x XL-density devices
//#define BOOTLOADER_FW_ADDRESS   0x1FFFE000
// stm32f10x devices
#define BOOTLOADER_FW_ADDRESS   0x1FFFF000

void checkDfuAndJump()
{
    uint8_t val;
    if (SharedParamsReadByIndex(0, &val) == true) {
        if (val == 0x02) {
            // reset flag
            SharedParamsWriteByIndex(0, 0x00);

            // AN2606 says: 2 Kbytes, starting from address 0x1FFFF000 contain the bootloader firmware.
            // Point the PC to the System Memory reset vector (+4)
            void (*SysMemBootJump)(void) = (void (*)(void)) (*((uint32_t *) (BOOTLOADER_FW_ADDRESS + 4)));
            // Pick stack address from vector table
            __set_MSP(*(__IO uint32_t*) BOOTLOADER_FW_ADDRESS);
            SysMemBootJump();
            while (1);
        }
    }
}

void ToggleESRDriver(SensorType sensor)
{
    switch (sensor) {
        case SensorType::LSU42:
            palTogglePad(NERNST_42_ESR_DRIVER_PORT, NERNST_42_ESR_DRIVER_PIN);
        break;
        case SensorType::LSU49:
            palTogglePad(NERNST_49_ESR_DRIVER_PORT, NERNST_49_ESR_DRIVER_PIN);
        break;
        case SensorType::LSUADV:
            palTogglePad(NERNST_ADV_ESR_DRIVER_PORT, NERNST_ADV_ESR_DRIVER_PIN);
        break;
    }
}