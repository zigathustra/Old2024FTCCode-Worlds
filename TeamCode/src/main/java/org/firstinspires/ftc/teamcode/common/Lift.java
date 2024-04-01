package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.config.Constants;

@Config
public class Lift extends Component{
    private final DcMotorEx liftMotorL;
    private final DcMotorEx liftMotorR;

    private final double kP = 0.0;
    private final double kI = 0.0;
    private final double kD = 0.0;
    private final double ff = 0.0;

    private final Telemetry telemetry;
    private final int retractPos = 50;
    private final int deployPos = 300;
    private final int maxPos = 1000;
    private final int minPos = 200;
    private final int defaultPosition = 250;
    private int targetPos = defaultPosition;

    private final double defaultMaxSpeed = 0.5;
    private double maxSpeed = defaultMaxSpeed;

    public Lift(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

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

    public void update()
    {
        //setPIDMotorPower();
    }
    public void up(double speed) {
        maxSpeed = speed;
        if (!atTop()) {
            targetPos++;
        }
        logPosition();
    }

    public void down(double speed) {
        maxSpeed = speed;
        if (!atBottom()) {
            targetPos--;
        }
        logPosition();
    }

    public void stop()
    {
        maxSpeed = 0;
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

    public void setTargetPos(int targetPos)
    {
        if (targetPos >= minPos && targetPos <= maxPos) {
            this.targetPos = targetPos;
        }
    }

    private void setLiftMotorPower()
    {
//        double power = pid.calculate(targetPos, liftMotorL.getCurrentPosition()) + ff;
//        liftMotorL.setPower(power);
//        liftMotorR.setPower(power);
    }
        private void logPosition()
        {
            telemetry.addData("PositionL:  ", liftMotorL.getCurrentPosition());
            telemetry.addData("PositionR:  ", liftMotorR.getCurrentPosition());

            telemetry.update();
        }

    }