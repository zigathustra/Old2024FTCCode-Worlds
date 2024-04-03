package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drivetrain {
    protected Telemetry telemetry;
    public final DcMotorEx leftFrontDrive;
    public final DcMotorEx rightFrontDrive;
    public final DcMotorEx leftBackDrive;
    public final DcMotorEx rightBackDrive;
    protected final double maxNormalPowerFactor = 0.75;
    protected final double maxSlowedPowerFactor = 0.5;
    protected final double maxCreepPowerFactor = 0.35;
    protected double maxPowerFactor =  maxNormalPowerFactor;
    protected boolean loggingOn;

    protected Drivetrain(HardwareMap hardwareMap, Telemetry telemetry, boolean loggingOn)
    {
        this.telemetry = telemetry;
        this.loggingOn = loggingOn;

        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "leftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightBackDrive");

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        setBrakingOn();
    }
    public void setToNormalPowerFactor()
    {
        maxPowerFactor = maxNormalPowerFactor;
    }

    public void setToSlowedPowerFactor(){
        maxPowerFactor = maxSlowedPowerFactor;
    }
    protected void setBrakingOn() {
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    protected void setBrakingOff() {
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
}
