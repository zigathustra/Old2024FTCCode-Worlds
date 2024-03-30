package org.firstinspires.ftc.teamcode.common.config;
public class GoBilda435DcMotorData {
    public static double countsPerMotorRev = 28;
    public static double gearRatio = 13.7;
    public static double wheelDiameterInches = 96.0/25.4;
    public static double liftPulleyDiameterInches = 44.0/25.4;
    public static double countsPerGearboxRev = gearRatio * countsPerMotorRev;
    public static double wheelCircumferenceInches = wheelDiameterInches * Math.PI;
    public static double liftPulleyCircumferenceInches = liftPulleyDiameterInches * Math.PI;
    public static double wheelCountsPerInch = countsPerGearboxRev/wheelCircumferenceInches;
    public static double liftPulleyCountsPerInch = countsPerGearboxRev/liftPulleyCircumferenceInches;
    public static double maxMotorRpm = 5900;
    public static double maxMotorRps = maxMotorRpm / 60.0;
    public static double maxCountsPerSec = maxMotorRps * countsPerMotorRev;
}
