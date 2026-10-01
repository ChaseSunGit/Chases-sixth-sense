#include "Config.h"
#include "ICM42607_Driver.h"
#include "MMC5983MA_Driver.h"

Preferences prefs;

void Storage_Init() {
    prefs.begin("calib_data", false); // false = R/W mode
}

void Storage_Save_Calibration() {
      // Save ICM Offsets
      prefs.putFloat("ax_off", ICM_Data_Holder.ax_offset);
      prefs.putFloat("ay_off", ICM_Data_Holder.ay_offset);
      prefs.putFloat("az_off", ICM_Data_Holder.az_offset);
      prefs.putFloat("gx_off", ICM_Data_Holder.gx_offset);
      prefs.putFloat("gy_off", ICM_Data_Holder.gy_offset);
      prefs.putFloat("gz_off", ICM_Data_Holder.gz_offset);

      // Save MMC Offsets
      prefs.putBytes("mmc_off", MMC_Data_Holder.offset, sizeof(MMC_Data_Holder.offset));
      prefs.putBytes("mmc_w", MMC_Data_Holder.W, sizeof(MMC_Data_Holder.W));
}

void Storage_Load_Calibration() {
      // Load ICM Offsets
      ICM_Data_Holder.ax_offset = prefs.getFloat("ax_off", 0.0f);
      ICM_Data_Holder.ay_offset = prefs.getFloat("ay_off", 0.0f);
      ICM_Data_Holder.az_offset = prefs.getFloat("az_off", 0.0f);
      ICM_Data_Holder.gx_offset = prefs.getFloat("gx_off", 0.0f);
      ICM_Data_Holder.gy_offset = prefs.getFloat("gy_off", 0.0f);
      ICM_Data_Holder.gz_offset = prefs.getFloat("gz_off", 0.0f);

      // Load MMC Offsets
      if (prefs.getBytesLength("mmc_off") == sizeof(MMC_Data_Holder.offset)) {
            prefs.getBytes("mmc_off", MMC_Data_Holder.offset, sizeof(MMC_Data_Holder.offset));
            prefs.getBytes("mmc_w", MMC_Data_Holder.W, sizeof(MMC_Data_Holder.W));
      }
}