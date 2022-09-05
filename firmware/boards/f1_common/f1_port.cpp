#include "port.h"

#include "wideband_config.h"

#include "hal.h"
#include "hal_mfs.h"

// Storage

static const MFSConfig mfscfg1 = {
    .flashp           = (BaseFlash *)&EFLD1,
    .erased           = 0xFFFFFFFFU,
    .bank_size        = 4096U,
    .bank0_start      = 120U,
    .bank0_sectors    = 4U,
    .bank1_start      = 124U,
    .bank1_sectors    = 4U
};

static MFSDriver mfs1;

// Settings
static Configuration cfg;
#define MFS_CONFIGURATION_RECORD_ID     1

// Configuration defaults
void Configuration::LoadDefaults()
{
    CanIndexOffset = 0;

#if 0
    // https://rusefi.com/forum/viewtopic.php?f=4&t=2410 for now
    auxInput[0] = 0; // AFR0
    auxOutBins[0]   = { 10.01, 11.01, 12.01, 13.01, 14.01, 15.01, 16.01, 20.01 };
    auxOutValues[0] = {  1.00,  0.90,  0.80,  0.70,  0.60,  0,50,  0.40,  0.00 };

    // Second output emulated narrow band AFR
    // see http://www.edproject.co.uk/NarrowAFR.html

#endif

    /* Finaly */
    Tag = ExpectedTag;
}

int InitConfiguration()
{
    size_t size = GetConfiguratiuonSize();

    /* Starting EFL driver.*/
    eflStart(&EFLD1, NULL);

    mfsObjectInit(&mfs1);

    mfsStart(&mfs1, &mfscfg1);

    mfs_error_t err = mfsReadRecord(&mfs1, MFS_CONFIGURATION_RECORD_ID, &size, GetConfiguratiuonPtr());
    if ((err != MFS_NO_ERROR) || (size != GetConfiguratiuonSize() || !cfg.IsValid())) {
        /* load defaults */
        cfg.LoadDefaults();
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
void SaveConfiguration() {
    /* TODO: handle error */
    mfsWriteRecord(&mfs1, MFS_CONFIGURATION_RECORD_ID, GetConfiguratiuonSize(), GetConfiguratiuonPtr());
}

uint8_t *GetConfiguratiuonPtr()
{
    return (uint8_t *)&cfg;
}

size_t GetConfiguratiuonSize()
{
    return sizeof(cfg);
}

const char *getTsSignature() {
    return TS_SIGNATURE;
}
