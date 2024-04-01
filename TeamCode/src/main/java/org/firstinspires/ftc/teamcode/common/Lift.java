package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.dashboard.config.Config;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.config.Constants;
import org.firstinspires.ftc.teamcode.common.config.GoBilda435DcMotorData;

@Config
public class Lift extends Component {
    private final DcMotorEx liftMotorL;
    private final DcMotorEx liftMotorR;
    private final PIDFController pidf;
    //    private final PIDFController pidfR;
    private final double kP = 0.0;
    private final double kI = 0.0;
    private final double kD = 0.0;
    private final double kF = 0.0;
    private double maxVelocity = GoBilda435DcMotorData.maxCountsPerSec;
    private final Telemetry telemetry;
    private final int retractPos = 50;
    private final int deployPos = 300;
    private final int maxPos = 1000;
    private final int minPos = 200;
    private final int defaultPosition = 250;
    private int targetPos = defaultPosition;
    private final double defaultSpeedFactor = 0.5;
    private double speedFactor = defaultSpeedFactor;
    private double speedL = 0.0;
    private double speedR = 0.0;

    public Lift(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        pidf = new PIDFController(kP, kI, kD, kF);

        liftMotorL = hardwareMap.get(DcMotorEx.class, "liftLeft");
        liftMotorR = hardwareMap.get(DcMotorEx.class, "liftRight");

        liftMotorL.setDirection(DcMotor.Direction.FORWARD);
        liftMotorR.setDirection(DcMotor.Direction.REVERSE);

        liftMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setTargetPos(retractPos);
    }

    public void update() {
        setPIDFMotorVelocity();
    }

    public void up(double speedFactor) {
        this.speedFactor = speedFactor;
        if (!atTop()) {
            targetPos++;
        }
        logPosition();
    }

    public void down(double speed) {
        this.speedFactor = speedFactor;
        if (!atBottom()) {
            targetPos--;
        }
        logPosition();
    }

    private boolean atTop() {
        if (liftMotorL.getCurrentPosition() >= maxPos) {
            return true;
        } else {
            return false;
        }
    }

    private boolean atBottom() {
        if (liftMotorL.getCurrentPosition() <= minPos) {
            return true;
        } else {
            return false;
        }
    }

    public void setTargetPos(int targetPos) {
        if (targetPos >= minPos && targetPos <= maxPos) {
            this.targetPos = targetPos;
        }
    }

    private int avgCurrentPos() {
        int currentPos, currentPosL, currentPosR = 0;
        currentPosL = liftMotorL.getCurrentPosition();
        currentPosR = liftMotorR.getCurrentPosition();
        currentPos = (int) ((currentPosL + currentPosR) / 2.0);
        return (currentPos);
    }

    private void setPIDFMotorVelocity() {
        double velocity = 0.0;
        if (!pidf.atSetPoint()) {
            velocity = pidf.calculate(avgCurrentPos(), targetPos);
        }
        liftMotorL.setVelocity(velocity * speedFactor);
        liftMotorR.setVelocity(velocity * speedFactor);
    }

    private void logPosition() {
        telemetry.addData("PositionL:  ", liftMotorL.getCurrentPosition());
        telemetry.addData("PositionR:  ", liftMotorR.getCurrentPosition());

        telemetry.update();
    }

}