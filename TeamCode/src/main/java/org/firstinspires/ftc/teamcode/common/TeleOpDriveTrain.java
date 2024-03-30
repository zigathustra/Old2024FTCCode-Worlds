package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.config.Constants;

public class TeleOpDriveTrain{
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightBackDrive = null;
    private double maxSpeed = Constants.maxTeleOpNormalSpeed;
//    private IMU imu = null;
    LinearOpMode opMode = null;

    public TeleOpDriveTrain(LinearOpMode opMode) {
        this.opMode = opMode;

        leftFrontDrive = opMode.hardwareMap.get(DcMotorEx.class, "leftFrontDrive");
        leftBackDrive = opMode.hardwareMap.get(DcMotorEx.class, "leftBackDrive");
        rightFrontDrive = opMode.hardwareMap.get(DcMotorEx.class, "rightFrontDrive");
        rightBackDrive = opMode.hardwareMap.get(DcMotorEx.class, "rightBackDrive");

        leftFrontDrive.setDirection(Constants.drivetrainLeftFrontDriveDirection);
        leftBackDrive.setDirection(Constants.drivetrainLeftBackDriveDirection);
        rightFrontDrive.setDirection(Constants.drivetrainRightFrontDriveDirection);
        rightBackDrive.setDirection(Constants.drivetrainRightBackDriveDirection);

//        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
//        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
//        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

//        imu = opMode.hardwareMap.get(IMU.class, "imu");
//        imu.initialize(new IMU.Parameters(orientationOnRobot));
//        imu.resetYaw();

        setBrakingOn();
    }

    public void setToNormalSpeed()
    {
        this.maxSpeed = Constants.maxTeleOpNormalSpeed;
    }

    public void setToSlowedSpeed()
    {
        this.maxSpeed = Constants.maxTeleOpSlowedSpeed;
    }

    public void creepDirection(double axial, double strafe, double yaw) {
        moveDirection(axial * Constants.maxTeleOpCreepSpeed, strafe * Constants.maxTeleOpCreepSpeed, yaw * Constants.maxTeleOpCreepSpeed);
    }

    public void moveDirection(double axial, double strafe, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower = axial - strafe - yaw;
        double rightFrontPower = axial + strafe + yaw;
        double leftBackPower = axial + strafe - yaw;
        double rightBackPower = axial - strafe + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFrontDrive.setPower(leftFrontPower * maxSpeed);
        rightFrontDrive.setPower(rightFrontPower * maxSpeed);
        leftBackDrive.setPower(leftBackPower * maxSpeed);
        rightBackDrive.setPower(rightBackPower * maxSpeed);
    }

//    public double getHeading() {
//        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//    }

    public void stop() {
        leftFrontDrive.setPower(0.0);
        leftBackDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        rightBackDrive.setPower(0.0);
    }

    private void setBrakingOn() {
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void setBrakingOff() {
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
}