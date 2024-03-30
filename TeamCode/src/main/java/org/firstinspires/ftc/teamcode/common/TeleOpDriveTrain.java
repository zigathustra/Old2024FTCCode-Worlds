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
    private DcMotorEx leftRearDrive = null;
    private DcMotorEx rightRearDrive = null;
    private double maxSpeed = Constants.maxTeleOpSpeedNormal;
    private IMU imu = null;
    LinearOpMode opMode = null;

    public TeleOpDriveTrain(LinearOpMode opMode) {
        this.opMode = opMode;

        leftFrontDrive = opMode.hardwareMap.get(DcMotorEx.class, "leftFrontDrive");
        leftRearDrive = opMode.hardwareMap.get(DcMotorEx.class, "leftRearDrive");
        rightFrontDrive = opMode.hardwareMap.get(DcMotorEx.class, "rightFrontDrive");
        rightRearDrive = opMode.hardwareMap.get(DcMotorEx.class, "rightRearDrive");

        leftFrontDrive.setDirection(Constants.drivetrainLeftFrontDirection);
        leftRearDrive.setDirection(Constants.drivetrainLeftRearDirection);
        rightFrontDrive.setDirection(Constants.drivetrainRightFrontDirection);
        rightRearDrive.setDirection(Constants.drivetrainRightRearDirection);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = opMode.hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        setBrakingOn();
    }

    public void setToNormalSpeed()
    {
        this.maxSpeed = Constants.maxTeleOpSpeedNormal;
    }

    public void setToSlowedSpeed()
    {
        this.maxSpeed = Constants.maxTeleOpSpeedSlowed;
    }

    public void creepDirection(double axial, double strafe, double yaw) {
        moveDirection(axial * Constants.maxTeleOpCreepSpeed, strafe * Constants.maxTeleOpCreepSpeed, yaw * Constants.maxTeleOpCreepSpeed);
    }

    public void moveDirection(double axial, double strafe, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower = axial - strafe - yaw;
        double rightFrontPower = axial + strafe + yaw;
        double leftRearPower = axial + strafe - yaw;
        double rightRearPower = axial - strafe + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftRearPower));
        max = Math.max(max, Math.abs(rightRearPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftRearPower /= max;
            rightRearPower /= max;
        }

        leftFrontDrive.setPower(leftFrontPower * maxSpeed);
        rightFrontDrive.setPower(rightFrontPower * maxSpeed);
        leftRearDrive.setPower(leftRearPower * maxSpeed);
        rightRearDrive.setPower(rightRearPower * maxSpeed);
    }
    
    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void stop() {
        leftFrontDrive.setPower(0.0);
        leftRearDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        rightRearDrive.setPower(0.0);
    }

    private void setBrakingOn() {
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void setBrakingOff() {
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
}