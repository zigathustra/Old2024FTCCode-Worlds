package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.config.Constants;

public class AutoDriveTrain extends DriveTrain{
    private IMU imu = null;
    public AutoDriveTrain(LinearOpMode opMode, double maxSpeed) {
        super(opMode, maxSpeed);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = opMode.hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }

    public void turnToHeading(double targetHeading) {
        double turnSpeed = Constants.maxAutoCorrectionTurnSpeed;
        double headingError = getHeadingError(targetHeading);

        // keep looping while we are still active, and not on heading.
        while (Math.abs(headingError) > Constants.autoHeadingThreshold) {
            headingError = getHeadingError(targetHeading);

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(headingError, Constants.autoTurnGain);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -Constants.maxAutoCorrectionTurnSpeed, Constants.maxAutoCorrectionTurnSpeed);

            // Pivot in place by applying the turning correction
            moveDirection(0, 0, turnSpeed);
        }
        stop();
    }

    public void turnForDistance(double distance) {
        double targetHeading = getHeading() + distance;
        turnToHeading(targetHeading);
    }

    public void creepDirection(double axial, double strafe, double yaw) {
        moveDirection(axial * Constants.maxCreepSpeed, strafe * Constants.maxCreepSpeed, yaw * Constants.maxCreepSpeed);
    }

    public void strafeForDistance(double distance) {
        int targetCounts = (int) (distance * strafeCountsPerInch);
        int leftFrontTarget = 0;
        int leftRearTarget = 0;
        int rightFrontTarget = 0;
        int rightRearTarget = 0;
        double strafeSpeed = Constants.maxAutoStrafeSpeed;

        leftFrontTarget = leftFrontDrive.getCurrentPosition() + targetCounts;
        leftRearTarget = leftRearDrive.getCurrentPosition() - targetCounts;
        rightFrontTarget = rightFrontDrive.getCurrentPosition() - targetCounts;
        rightRearTarget = rightRearDrive.getCurrentPosition() + targetCounts;

        leftFrontDrive.setTargetPosition(leftFrontTarget);
        leftRearDrive.setTargetPosition(leftRearTarget);
        rightFrontDrive.setTargetPosition(rightFrontTarget);
        rightRearDrive.setTargetPosition(rightRearTarget);

        setRunToPosition();

        while (leftFrontDrive.isBusy() && leftRearDrive.isBusy() && rightFrontDrive.isBusy() && rightRearDrive.isBusy() && !opMode.isStopRequested()) {
            moveDirection(0, strafeSpeed, 0);
        }
        stop();
        setRunUsingEncoder();
    }

    public void moveStraightForDistance(double distance) {
        moveStraightForDistance(distance, Constants.maxAutoCorrectionTurnSpeed, Constants.maxAutoCorrectionDriveSpeed);
    }

    public void creepStraightForDistance(double distance) {
        moveStraightForDistance(distance, Constants.maxCreepSpeed, Constants.maxCreepSpeed);
    }

    private void moveStraightForDistance(double distance, double turnSpeed, double driveSpeed) {
        int targetCounts = (int) (distance * moveCountsPerInch);
        int leftFrontTarget = 0;
        int leftRearTarget = 0;
        int rightFrontTarget = 0;
        int rightRearTarget = 0;
        double headingError = 0;
        double targetHeading = getHeading();

        leftFrontTarget = leftFrontDrive.getCurrentPosition() + targetCounts;
        leftRearTarget = leftRearDrive.getCurrentPosition() + targetCounts;
        rightFrontTarget = rightFrontDrive.getCurrentPosition() + targetCounts;
        rightRearTarget = rightRearDrive.getCurrentPosition() + targetCounts;

        leftFrontDrive.setTargetPosition(leftFrontTarget);
        leftRearDrive.setTargetPosition(leftRearTarget);
        rightFrontDrive.setTargetPosition(rightFrontTarget);
        rightRearDrive.setTargetPosition(rightRearTarget);

        setRunToPosition();

        while (leftFrontDrive.isBusy() && leftRearDrive.isBusy() && rightFrontDrive.isBusy() && rightRearDrive.isBusy() && !opMode.isStopRequested()) {
            //while (leftFrontDrive.isBusy()) {
            headingError = getHeadingError(targetHeading);
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(headingError, Constants.autoDriveGain);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                turnSpeed *= -1.0;

            // Apply the turning correction to the current driving speed.
            moveDirection(driveSpeed, 0.0, turnSpeed);
        }
        stop();
        setRunUsingEncoder();
    }

    public double getHeadingError(double targetHeading) {
        return (targetHeading - getHeading());
    }

    public double getSteeringCorrection(double headingError, double gain) {
        // Determine the heading current error

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * gain, -1, 1);
    }

    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}
