#include "port.h"
#include "shared/strap_pin.h"

#include "wideband_config.h"

#include "hal.h"
#include "hal_mfs.h"

#if USE_OPENBLT
/* communication with OpenBLT that is plain C, not to modify external file */
extern "C" {
    #include "openblt/shared_params.h"
};
#endif

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

static size_t boardHwId = 0;

size_t BoardGetHwId()
{
    return boardHwId;
}

int InitConfiguration()
{
    // See https://github.com/mck1117/wideband/issues/11 to explain this madness
    auto sel1 = readSelPin(ID_SEL1_PORT, ID_SEL1_PIN);
#ifdef ID_SEL2_PORT
    auto sel2 = readSelPin(ID_SEL2_PORT, ID_SEL2_PIN);
#else
    int sel2 = 0;
#endif

    boardHwId = (3 * sel1 + sel2);

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
        cfg.LoadDefaults();

        // TODO: override defaults with a hardware-strapped options
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
#if USE_OPENBLT
    /* safe to call on already inited shares area */
    SharedParamsInit();
    /* Store flag to stay in OpenBLT */
    SharedParamsWriteByIndex(0, 0x01);

    rebootNow();
#endif
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