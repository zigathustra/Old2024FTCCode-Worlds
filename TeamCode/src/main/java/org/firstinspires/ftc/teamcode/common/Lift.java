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
    private final PIDFController pidfL;
    private final PIDFController pidfR;
    public static double kP = 0.03;
    public static double kI = 0.0;
    public static double kD = 0.001;
    public static double kF = 0.00037;
    private final double positionTolerance = 10;
    private final double derivativeTolerance = 10;
    private final double maxVelocity = GoBilda435DcMotorData.maxTicksPerSec;
    private final int maxPos = 2000;
    private final int retractPos = 800;
    private final int deployPos = 1000;
    private final int minPos = 0;
    public static int targetPos = 250;
    private final double defaultMaxPower = 1.0;
    private double maxPower = defaultMaxPower;
//    private final int increment = 50;
    public static int currentPos;


    public Lift(HardwareMap hardwareMap, Telemetry telemetry, boolean loggingOn) {
        super(telemetry, loggingOn);
        pidfL = new PIDFController(kP, kI, kD, kF);
        pidfL.setTolerance(positionTolerance);
        pidfR = new PIDFController(kP, kI, kD, kF);
        pidfR.setTolerance(positionTolerance);

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
        pidfL.setPIDF(kP,kI,kD,kF);
        pidfR.setPIDF(kP,kI,kD,kF);

        setPIDFMotorPower();
        if (loggingOn) {
            logTelemetry();
        }
    }

    /*
    public void autoUp(double maxPower) {
        this.maxPower = maxPower;
        if (!atTop()) {
            targetPos = targetPos + increment;
        }
    }

    public void autoDown(double maxPower) {
        this.maxPower = maxPower;
        if (!atBottom()) {
            targetPos = targetPos - increment;
        }
    }
    */

    public void manualUp(double power)
    {
        if (!atTop())
        {
            setMotorsPower(power);
        } else
        {
            stop();
        }
    }

    public void manualDown(double power)
    {
        if (!atBottom())
        {
            setMotorsPower(power);
        } else
        {
            stop();
        }
    }


    public void stop()
    {
        setTargetPos(liftMotorL.getCurrentPosition());
    }
    public void goToRetractPosition()
    {
        setTargetPos(retractPos);
    }
    public void goToMinPosition() {
        setTargetPos(minPos);
    }
    public void goToDeployPosition()
    {
        setTargetPos(deployPos);
    }

    private boolean atTop() {
        if (liftMotorL.getCurrentPosition() >= maxPos) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isBusy() {
        return (!pidfL.atSetPoint() || !pidfR.atSetPoint());
    }

    private boolean atBottom() {
        if (liftMotorL.getCurrentPosition() <= minPos) {
            return true;
        } else {
            return false;
        }
    }

    public void setMotorsPower(double power) {
        liftMotorL.setPower(power * maxPower);
        liftMotorR.setPower(power * maxPower);
    }

    public void setLMotorPower(double power) {
        liftMotorL.setPower(power * maxPower);
    }

    public void setRMotorPower(double power) {
        liftMotorR.setPower(power * maxPower);
    }

    public void setTargetPos(int targetPos) {
        if (targetPos < minPos) {
            this.targetPos = minPos;
        } else if (targetPos > maxPos)
        {
            this.targetPos = maxPos;
        } else
        {
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
        setLMotorPower(pidfL.calculate(liftMotorL.getCurrentPosition(), targetPos));
        setRMotorPower(pidfR.calculate(liftMotorR.getCurrentPosition(), targetPos));
    }

    private void logTelemetry() {
        telemetry.addData("PositionL:  ", liftMotorL.getCurrentPosition());
        telemetry.addData("PositionR:  ", liftMotorR.getCurrentPosition());
        telemetry.addData("Target:  ", targetPos);
        telemetry.addData("PowerL:  ", liftMotorL.getPower());
        telemetry.addData("PowerR:  ", liftMotorR.getPower());
        telemetry.addData("Busy:  ", isBusy());
        telemetry.update();
    }
    /*
    public class LiftUp implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                up(maxPower);
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