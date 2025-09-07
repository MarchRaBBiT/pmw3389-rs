//! PMW3389 register definitions (standard public registers from datasheet)
//! Only registers documented and generally useful are included.

/// PMW3389 register addresses and descriptions
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Register {
    // Identification
    ProductId              = 0x00, // Product_ID
    RevisionId             = 0x01, // Revision_ID

    // Motion and delta
    Motion                 = 0x02, // Motion
    DeltaX_L               = 0x03, // Delta_X_L
    DeltaX_H               = 0x04, // Delta_X_H
    DeltaY_L               = 0x05, // Delta_Y_L
    DeltaY_H               = 0x06, // Delta_Y_H

    // Observation / Surface metrics
    Squal                  = 0x07, // SQUAL
    RawData_Sum            = 0x08, // RawData_Sum
    Maximum_RawData        = 0x09, // Maximum_RawData
    Minimum_RawData        = 0x0A, // Minimum_RawData
    Shutter_Lower          = 0x0B, // Shutter_Lower
    Shutter_Upper          = 0x0C, // Shutter_Upper

    // Resolution / CPI
    Ripple_Control         = 0x0D, // Ripple Control
    Resolution_L           = 0x0E, // Resolution_L
    Resolution_H           = 0x0F, // Resolution_H

    // Configuration
    Config2                = 0x10, // Config2
    Angle_Tune             = 0x11, // Angle_Tune
    Frame_Capture          = 0x12, // Frame_Capture
    Srom_Enable            = 0x13, // SROM_Enable
    Run_Downshift          = 0x14, // Run_Downshift
    Rest1_Rate_Lower       = 0x15, // Rest1_Rate_Lower
    Rest1_Rate_Upper       = 0x16, // Rest1_Rate_Upper
    Rest1_Downshift        = 0x17, // Rest1_Downshift
    Rest2_Rate_Lower       = 0x18, // Rest2_Rate_Lower
    Rest2_Rate_Upper       = 0x19, // Rest2_Rate_Upper
    Rest2_Downshift        = 0x1A, // Rest2_Downshift
    Rest3_Rate_Lower       = 0x1B, // Rest3_Rate_Lower
    Rest3_Rate_Upper       = 0x1C, // Rest3_Rate_Upper

    Observation            = 0x24, // Observation
    Data_Out_Lower         = 0x25, // Data_Out_Lower
    Data_Out_Upper         = 0x26, // Data_Out_Upper

    Min_SQ_Run             = 0x2B, // Min_SQ_Run
    RawData_Threshold      = 0x2C, // RawData_Threshold
    Control2               = 0x2D, // Control2
    Config5_L              = 0x2E, // Config5_L
    Config5_H              = 0x2F, // Config5_H

    SromId                 = 0x2A, // SROM_ID
    Motion_Burst           = 0x50, // Motion_Burst

    // Control registers
    Power_Up_Reset         = 0x3A, // Power_Up_Reset
    Shutdown               = 0x3B, // Shutdown
    Inverse_Product_ID     = 0x3F, // Inverse_Product_ID
    Angle_Snap             = 0x42, // Angle_Snap

    LiftCutoff_Cal3        = 0x41, // LiftCutoff_Cal3
    LiftCutoff_Cal1        = 0x4A, // LiftCutoff_Cal1
    Lift_Config            = 0x63, // Lift_Config
}

impl Register {
    /// Return the raw register address as u8
    #[inline]
    pub fn addr(self) -> u8 {
        self as u8
    }
}
