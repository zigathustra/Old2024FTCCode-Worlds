package org.firstinspires.ftc.teamcode.common.config;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Constants {
    // April Tag Detection Settings
    public static final double atAxialGain = 0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    public static final double atStrafeGain = 0.02;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    public static final double atYawGain = 0.02;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    public static final double atMaxAxial = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    public static final double atMaxStrafe = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    public static final double atMaxYaw = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    public static final int atExposureMS = 6;
    public static final int atExposureGain = 250;
}
