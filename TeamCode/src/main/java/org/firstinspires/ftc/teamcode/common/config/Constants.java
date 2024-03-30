package org.firstinspires.ftc.teamcode.common.config;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Constants {
    // Drivetrain Settings
    public static final double driveTrainMaxVelocity = GoBilda312DcMotorData.maxCountsPerSec;
    public static final double mecanumMoveFactor = 1.0;
    public static final double mecanumMoveCountsPerInch = mecanumMoveFactor * GoBilda312DcMotorData.wheelCountsPerInch;
    public static final double mecanumStrafeFactor = 1.0;
    public static final double mecanumStrafeCountsPerInch = mecanumStrafeFactor * GoBilda312DcMotorData.wheelCountsPerInch;
    public static final DcMotor.Direction drivetrainLeftFrontDriveDirection = DcMotor.Direction.REVERSE;
    public static final DcMotor.Direction drivetrainLeftBackDriveDirection = DcMotor.Direction.REVERSE;
    public static final DcMotor.Direction drivetrainRightFrontDriveDirection = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction drivetrainRightBackDriveDirection = DcMotor.Direction.FORWARD;
    public static final double maxTeleOpNormalSpeed = 0.75;
    public static final double maxTeleOpSlowedSpeed = 0.5;
    public static final double maxTeleOpCreepSpeed = 0.15;

    // RoadRunner Drive Constants

    /*
     * These are motor constants that should be listed online for your motors.
     */
    // Values for GoBilda 312 RPM
    public static final double TICKS_PER_REV = 537.7;
    public static final double MAX_RPM = 312;

    /*
     * Set RUN_USING_ENCODER to true to enable built-in hub velocity control using drive encoders.
     * Set this flag to false if drive encoders are not present and an alternative localization
     * method is in use (e.g., tracking wheels).
     *
     * If using the built-in motor velocity PID, update MOTOR_VELO_PID with the tuned coefficients
     * from DriveVelocityPIDTuner.
     */

    public static final boolean RUN_USING_ENCODER = false;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
            getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

    /*
     * These are physical constants that can be determined from your robot (including the track
     * width; it will be tune empirically later although a rough estimate is important). Users are
     * free to chose whichever linear distance unit they would like so long as it is consistently
     * used. The default values were selected with inches in mind. Road runner uses radians for
     * angular distances although most angular parameters are wrapped in Math.toRadians() for
     * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
     */
    public static double WHEEL_RADIUS = 1.88976; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 16.34; // in

    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */
    public static double kV = 1.0 / rpmToVelocity(MAX_RPM);
    public static double kA = 0;
    public static double kStatic = 0;

    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. All distance units are
     * inches.
     */
    /*
     * Note from LearnRoadRunner.com:
     * The velocity and acceleration constraints were calculated based on the following equation:
     * ((MAX_RPM / 60) * GEAR_RATIO * WHEEL_RADIUS * 2 * Math.PI) * 0.85
     * Resulting in 52.48180821614297 in/s.
     * This is only 85% of the theoretical maximum velocity of the bot, following the recommendation above.
     * This is capped at 85% because there are a number of variables that will prevent your bot from actually
     * reaching this maximum velocity: voltage dropping over the game, bot weight, general mechanical inefficiencies, etc.
     * However, you can push this higher yourself if you'd like. Perhaps raise it to 90-95% of the theoretically
     * max velocity. The theoretically maximum velocity is 61.74330378369762 in/s.
     * Just make sure that your bot can actually reach this maximum velocity. Path following will be detrimentally
     * affected if it is aiming for a velocity not actually possible.
     *
     * The maximum acceleration is somewhat arbitrary and it is recommended that you tweak this yourself based on
     * actual testing. Just set it at a reasonable value and keep increasing until your path following starts
     * to degrade. As of now, it simply mirrors the velocity, resulting in 52.48180821614297 in/s/s
     *
     * Maximum Angular Velocity is calculated as: maximum velocity / trackWidth * (180 / Math.PI) but capped at 360°/s.
     * You are free to raise this on your own if you would like. It is best determined through experimentation.

     */
    public static double MAX_VEL = 52.48180821614297;
    public static double MAX_ACCEL = 52.48180821614297;
    public static double MAX_ANG_VEL = Math.toRadians(184.02607784577722);
    public static double MAX_ANG_ACCEL = Math.toRadians(184.02607784577722);


    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }


    // Lift Settings
    public static final double liftMaxVelocity = GoBilda435DcMotorData.maxCountsPerSec;
    public static final double liftMaxMoveSpeed = 1.0;
    public static final double liftStopPowerFactor = 1.0;
    public static final int liftMaxPosition = 1250;
    public static final int liftMaxTolerance = 25;
    public static final int liftMinPosition = 0;
    public static final int liftMinTolerance = 25;

    // Wrist settings
    public static final double wristDownPosition = 1;
    public static final double wristMiddlePosition = 0.5;
    public static final double wristUpPosition = 0.0;

    // Shoulder settings
    public static final double shoulderDownPosition = 1;
    public static final double shoulderMiddlePosition = 0.5;
    public static final double shoulderUpPosition = 0.0;


    // Dropper
    public static final double dropper1ClosedPosition = 0.0;
    public static final double dropper1OpenPosition = 1.0;
    public static final double dropper2ClosedPosition = 0.0;
    public static final double dropper2OpenPosition = 1.0;

    // Launcher settings
    public static final double launcherLockedPosition = 1;
    public static final double launcherUnlockedPosition = 0.55;

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
