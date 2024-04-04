package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.dashboard.config.Config;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.hardware_data.GoBilda435DcMotorData;

@Config
public class Lift extends Component {
    private final DcMotorEx liftMotorL;
    private final DcMotorEx liftMotorR;
    private final PIDFController pidf;
    //    private final PIDFController pidfR;
    public static double kP = 0.01;
    public static double kI = 0.0;
    public static double kD = 0.001;
    public static double kF = 0.0;
    private double positionTolerance = 10;
    private double derivativeTolerance = 10;
    public static double maxVelocity = GoBilda435DcMotorData.maxTicksPerSec;
    public static int maxPos = 1500;
    public static int retractPos = 800;
    private final int minPos = 0;
    public static final int defaultPosition = 250;
    public static int targetPos = defaultPosition;
    public static double defaultPowerFactor = 0.5;
    public static double powerFactor = defaultPowerFactor;
    public static boolean busy;
    public static double power = 0.0;
    public static int increment = 50;
    public static int currentPos;


    public Lift(HardwareMap hardwareMap, Telemetry telemetry, boolean loggingOn)
    {
        super(telemetry, loggingOn);
        pidf = new PIDFController(kP, kI, kD, kF);

        pidf.setTolerance(positionTolerance);

        liftMotorL = hardwareMap.get(DcMotorEx.class, "liftLeft");
        liftMotorR = hardwareMap.get(DcMotorEx.class, "liftRight");

        liftMotorL.setDirection(DcMotor.Direction.FORWARD);
        liftMotorR.setDirection(DcMotor.Direction.REVERSE);

        liftMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setTargetPos(retractPos);
    }

    public void update() {
        pidf.setD(kD);
        pidf.setF(kF);
        pidf.setI(kI);
        pidf.setP(kP);

        setPIDFMotorPower();
        if (loggingOn) {
            logTelemetry();
        }
    }

    public void up(double powerFactor) {
        this.powerFactor = powerFactor;
        if (!atTop()) {
            targetPos = targetPos + increment;
        }
    }

    public void down(double powerFactor) {
        this.powerFactor = powerFactor;
        if (!atBottom()) {
            targetPos = targetPos - increment;
        }
    }

    public void goToPositionSychronous(int target) {
        setTargetPos(target);
        update();
        while (isBusy()) {
            update();
        }
    }

    public void goToRetractPosition()
    {
        goToPositionSychronous(retractPos);
    }

    private boolean atTop() {
        if (liftMotorL.getCurrentPosition() >= maxPos) {
            return true;
        } else {
            return false;
        }
    }

    boolean isBusy() {
        return !pidf.atSetPoint();
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
        int currentPosL, currentPosR = 0;
        currentPosL = liftMotorL.getCurrentPosition();
        currentPosR = liftMotorR.getCurrentPosition();
        currentPos = (int) ((currentPosL + currentPosR) / 2.0);
        return (currentPos);
    }

    private void setPIDFMotorPower() {
        power = pidf.calculate(avgCurrentPos(), targetPos) * powerFactor;
        power = power * powerFactor;
        liftMotorL.setPower(power);
        liftMotorR.setPower(power);
    }

    private void logTelemetry() {
        telemetry.addData("PositionL:  ", liftMotorL.getCurrentPosition());
        telemetry.addData("PositionR:  ", liftMotorR.getCurrentPosition());
        telemetry.addData("Target:  ", targetPos);
        telemetry.addData("Power target: ", power);
        telemetry.addData("PowerL:  ", liftMotorL.getPower());
        telemetry.addData("PowerR:  ", liftMotorR.getPower());
        telemetry.update();
    }
    /*
    public class LiftUp implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                up(powerFactor);
                initialized = true;
            }

            double pos = lift.getCurrentPosition();
            packet.put("liftPos", pos);
            if (pos < 2500.0) {
                return true;
            } else {
                lift.setPower(0);
                return false;
            }
        }
    }
    public Action liftUp() {
        return new LiftUp();
    }
    */

}