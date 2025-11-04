//! PMW3389 register definitions (standard public registers from datasheet)
//! Only registers documented and generally useful are included.

/// PMW3389 register addresses and descriptions
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Register {
    // Identification
    ProductId = 0x00,  // Product_ID
    RevisionId = 0x01, // Revision_ID

    // Motion and delta
    Motion = 0x02,  // Motion
    DeltaXL = 0x03, // Delta_X_L
    DeltaXH = 0x04, // Delta_X_H
    DeltaYL = 0x05, // Delta_Y_L
    DeltaYH = 0x06, // Delta_Y_H

    // Observation / Surface metrics
    Squal = 0x07,          // SQUAL
    RawDataSum = 0x08,     // RawData_Sum
    MaximumRawData = 0x09, // Maximum_RawData
    MinimumRawData = 0x0A, // Minimum_RawData
    ShutterLower = 0x0B,   // Shutter_Lower
    ShutterUpper = 0x0C,   // Shutter_Upper

    // Resolution / CPI
    RippleControl = 0x0D, // Ripple Control
    ResolutionL = 0x0E,   // Resolution_L
    ResolutionH = 0x0F,   // Resolution_H

    // Configuration
    Config2 = 0x10,        // Config2
    AngleTune = 0x11,      // Angle_Tune
    FrameCapture = 0x12,   // Frame_Capture
    SromEnable = 0x13,     // SROM_Enable
    RunDownshift = 0x14,   // Run_Downshift
    Rest1RateLower = 0x15, // Rest1_Rate_Lower
    Rest1RateUpper = 0x16, // Rest1_Rate_Upper
    Rest1Downshift = 0x17, // Rest1_Downshift
    Rest2RateLower = 0x18, // Rest2_Rate_Lower
    Rest2RateUpper = 0x19, // Rest2_Rate_Upper
    Rest2Downshift = 0x1A, // Rest2_Downshift
    Rest3RateLower = 0x1B, // Rest3_Rate_Lower
    Rest3RateUpper = 0x1C, // Rest3_Rate_Upper

    Observation = 0x24,  // Observation
    DataOutLower = 0x25, // Data_Out_Lower
    DataOutUpper = 0x26, // Data_Out_Upper

    MinSQRun = 0x2B,         // Min_SQ_Run
    RawDataThreshold = 0x2C, // RawData_Threshold
    Control2 = 0x2D,         // Control2
    Config5L = 0x2E,         // Config5_L
    Config5H = 0x2F,         // Config5_H

    SromId = 0x2A,      // SROM_ID
    MotionBurst = 0x50, // Motion_Burst

    // Control registers
    PowerUpReset = 0x3A,     // Power_Up_Reset
    Shutdown = 0x3B,         // Shutdown
    InverseProductID = 0x3F, // Inverse_Product_ID
    AngleSnap = 0x42,        // Angle_Snap

    LiftCutoffCal3 = 0x41, // LiftCutoff_Cal3
    LiftCutoffCal1 = 0x4A, // LiftCutoff_Cal1
    LiftConfig = 0x63,     // Lift_Config

    // Self-test
    RunBist = 0x58,      // Run_BIST
    BistResult = 0x59,   // BIST_Result
}

impl Register {
    /// Return the raw register address as u8
    #[inline]
    pub fn addr(self) -> u8 {
        self as u8
    }
}
