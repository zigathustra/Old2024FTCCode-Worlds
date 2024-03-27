package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.common.config.Constants;

public class Lift {
    private final DcMotorEx liftMotorL, liftMotorR;
//    protected TouchSensor lift_sensor = null;
    private LinearOpMode opMode;

    public Lift(LinearOpMode opMode) {
        this.opMode = opMode;

//        lift_sensor = opMode.hardwareMap.get(TouchSensor.class, "lift_sensor");
        liftMotorL = opMode.hardwareMap.get(DcMotorEx.class, "liftL");
        liftMotorR = opMode.hardwareMap.get(DcMotorEx.class, "liftR");

        liftMotorL.setDirection(DcMotor.Direction.REVERSE);
        liftMotorR.setDirection(DcMotor.Direction.REVERSE);

        liftMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    private void logPosition()
    {
        opMode.telemetry.addData("PositionL:  ", liftMotorL.getCurrentPosition());
        opMode.telemetry.addData("PositionR:  ", liftMotorR.getCurrentPosition());

        opMode.telemetry.update();
    }
    public void stop() {
        stopAtPosition(liftMotorL.getCurrentPosition());
        logPosition();
    }

    public void liftUp(double targetSpeed) {
        if (!stoppedAtTop()) {
            liftMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            liftMotorL.setVelocity(targetSpeed * Constants.liftMaxMoveSpeed * Constants.liftMaxVelocity);
            liftMotorR.setVelocity(targetSpeed * Constants.liftMaxMoveSpeed * Constants.liftMaxVelocity);

            opMode.telemetry.addData("Stopped at Top: ", "true");
        } else {
            opMode.telemetry.addData("Stopped at Top: ", "false");
        }
        logPosition();
    }

    public void liftDown(double targetSpeed) {
        if (!stoppedAtBottom()) {
            liftMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            liftMotorL.setVelocity(-targetSpeed * Constants.liftMaxMoveSpeed * Constants.liftMaxVelocity);
            liftMotorR.setVelocity(-targetSpeed * Constants.liftMaxMoveSpeed * Constants.liftMaxVelocity);

            opMode.telemetry.addData("Stopped at Bottom: ", " true");
        } else {
            opMode.telemetry.addData("Stopped at Bottom: ", " false");
        }
        logPosition();
    }

    private boolean stoppedAtTop() {
        boolean stop = false;
        int currentPosition = liftMotorL.getCurrentPosition();
        if (currentPosition > (Constants.liftMaxPosition - Constants.liftMaxTolerance)) {
            stop = true;
            stopAtPosition(Constants.liftMaxPosition);
        }
        return stop;
    }

    private boolean stoppedAtBottom() {
        boolean stop = false;
        int currentPosition = liftMotorL.getCurrentPosition();
        if (currentPosition < (Constants.liftMinPosition - Constants.liftMinTolerance)) {
            stop = true;
            stopAtPosition(Constants.liftMinPosition);
        }
        return stop;
    }

    public void stopAtPosition(int targetPosition) {
        liftMotorL.setTargetPosition(targetPosition);
        liftMotorR.setTargetPosition(targetPosition);

        liftMotorL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        liftMotorR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        liftMotorL.setPower(Constants.liftStopPowerFactor);
        liftMotorR.setPower(Constants.liftStopPowerFactor);

        logPosition();
    }
    public void liftZero(){
//        liftDown(0.2);
//        while (!lift_sensor.isPressed()){
//        }
//        stop();
        //Move lift to zero position by detecting current spike
        liftMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}