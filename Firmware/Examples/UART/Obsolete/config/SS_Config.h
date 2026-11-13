#ifndef ICM_CONFIG_STORAGE_H
#define ICM_CONFIG_STORAGE_H

#include <Arduino.h>
#include <Preferences.h>
#include "ICM42607_Driver.h"

static Preferences imu_prefs;
static const char *NVS_NAMESPACE = "ICM_cfg";

/**
 * \brief Writes new calibration offsets and data rate to flash
 * \param ax_off Accelerometer X offset
 * \param ay_off Accelerometer Y offset
 * \param az_off Accelerometer Z offset
 * \param data_rate_reg Raw register setting or rate code (uint8_t)
 */
inline void ICM_flash_save_config(float ax_off, float ay_off, float az_off, uint8_t data_rate_reg) {
    // Open NVS namespace in Read/Write mode (false)
    if (!imu_prefs.begin(NVS_NAMESPACE, false)) {
        Serial.println("[NVS ERROR] Could not open namespa;'lce for writing.");
        return;
    }

    imu_prefs.putFloat("ax_off", ax_off);
    imu_prefs.putFloat("ay_off", ay_off);
    imu_prefs.putFloat("az_off", az_off);
    imu_prefs.putUChar("rate_cfg", data_rate_reg);
    imu_prefs.putBool("cal_ready", true); // Flag denoting valid data is present

    imu_prefs.end(); // Commits changes to SPI flash

    // Sync in-memory global state
    ICM_Data_Holder.ax_offset = ax_off;
    ICM_Data_Holder.ay_offset = ay_off;
    ICM_Data_Holder.az_offset = az_off;

    Serial.printf("[NVS] Stored Offsets: [%.4f, %.4f, %.4f] | Rate Reg: 0x%02X\n",
                  ax_off, ay_off, az_off, data_rate_reg);
}

/**
 * \brief Recalls stored constants from flash during startup
 * \param out_rate_reg Output reference to receive the stored uint8_t rate setting
 * \return True if valid calibration existed and was loaded, false otherwise
 */
inline bool ICM_flash_load_config(uint8_t &out_rate_reg) {
    // Open NVS namespace in Read-Only mode (true)
    if (!imu_prefs.begin(NVS_NAMESPACE, true)) {
        Serial.println("[NVS] No prior namespace found.");
        return false;
    }

    bool has_cal = imu_prefs.getBool("cal_ready", false);
    if (!has_cal) {
        Serial.println("[NVS] Flash contains no calibration records.");
        imu_prefs.end();
        return false;
    }

    // Read stored values directly into global holder and out parameter
    ICM_Data_Holder.ax_offset = imu_prefs.getFloat("ax_off", 0.0f);
    ICM_Data_Holder.ay_offset = imu_prefs.getFloat("ay_off", 0.0f);
    ICM_Data_Holder.az_offset = imu_prefs.getFloat("az_off", 0.0f);
    out_rate_reg              = imu_prefs.getUChar("rate_cfg", 0x00);

    imu_prefs.end();

    Serial.printf("[NVS] Loaded Offsets: [%.4f, %.4f, %.4f] | Rate Reg: 0x%02X\n",
                  ICM_Data_Holder.ax_offset,
                  ICM_Data_Holder.ay_offset,
                  ICM_Data_Holder.az_offset,
                  out_rate_reg);
    return true;
}

/**
 * \brief Optional: Erases stored calibration if you want to reset to defaults
 */
inline void ICM_flash_clear_config() {
    if (imu_prefs.begin(NVS_NAMESPACE, false)) {
        imu_prefs.clear();
        imu_prefs.end();
        Serial.println("[NVS] Calibration cleared.");
    }
}

#endif // ICM_CONFIG_STORAGE_H