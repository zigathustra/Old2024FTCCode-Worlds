package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.config.Constants;

public abstract class DriveTrain {
    protected DcMotorEx leftFrontDrive = null;
    protected DcMotorEx rightFrontDrive = null;
    protected DcMotorEx leftRearDrive = null;
    protected DcMotorEx rightRearDrive = null;
    protected double maxSpeed = 0.8; // Factor (0.0-1.0) to control drive speed
    protected double maxVelocity = Constants.driveTrainMaxVelocity;
    protected double moveCountsPerInch = Constants.mecanumMoveCountsPerInch;
    protected double strafeCountsPerInch = Constants.mecanumStrafeCountsPerInch;
    protected LinearOpMode opMode = null;

    public DriveTrain(LinearOpMode opMode, double maxSpeed) {
        this.opMode = opMode;
        this.maxSpeed = maxSpeed;

        leftFrontDrive = opMode.hardwareMap.get(DcMotorEx.class, "left_front_drive");
        leftRearDrive = opMode.hardwareMap.get(DcMotorEx.class, "left_rear_drive");
        rightFrontDrive = opMode.hardwareMap.get(DcMotorEx.class, "right_front_drive");
        rightRearDrive = opMode.hardwareMap.get(DcMotorEx.class, "right_rear_drive");

        leftFrontDrive.setDirection(Constants.drivetrainLeftFrontDirection);
        leftRearDrive.setDirection(Constants.drivetrainLeftRearDirection);
        rightFrontDrive.setDirection(Constants.drivetrainRightFrontDirection);
        rightRearDrive.setDirection(Constants.drivetrainRightRearDirection);

        stopAndResetEncoders();
        setRunUsingEncoder();
        setBrakingOn();
    }

    public void setToFastSpeed()
    {
        this.maxSpeed = Constants.maxNormalSpeed;
    }

    public void setToSlowedSpeed()
    {
        this.maxSpeed = Constants.maxSlowedSpeed;
    }

    public void creepDirection(double axial, double strafe, double yaw) {
        moveDirection(axial * Constants.maxCreepSpeed, strafe * Constants.maxCreepSpeed, yaw * Constants.maxCreepSpeed);
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

        leftFrontDrive.setVelocity(leftFrontPower * maxSpeed * maxVelocity);
        rightFrontDrive.setVelocity(rightFrontPower * maxSpeed * maxVelocity);
        leftRearDrive.setVelocity(leftRearPower * maxSpeed * maxVelocity);
        rightRearDrive.setVelocity(rightRearPower * maxSpeed * maxVelocity);
    }


    protected void stop() {
        leftFrontDrive.setVelocity(0.0);
        leftRearDrive.setVelocity(0.0);
        rightFrontDrive.setVelocity(0.0);
        rightRearDrive.setVelocity(0.0);
    }

    protected void setBrakingOn() {
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    protected void setBrakingOff() {
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    protected void stopAndResetEncoders() {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    protected void setRunWithoutEncoders() {
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    protected void setRunUsingEncoder() {
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    protected void setRunToPosition() {
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}