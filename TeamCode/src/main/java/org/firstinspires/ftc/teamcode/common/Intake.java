package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.config.Constants;

public class Intake {
    private final DcMotorEx intakeMotor;
    private final Servo intakeServo;
    private LinearOpMode opMode;

    public Intake(LinearOpMode opMode) {
        this.opMode = opMode;

        intakeMotor = opMode.hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeServo = opMode.hardwareMap.get(Servo.class, "intakeServo");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        raise();
        stop();
    }

    public void raise()
    {
        intakeServo.setPosition(0.85);
    }

    public void lower()
    {
        intakeServo.setPosition(0.35);
    }

    public void forward() {
        intakeMotor.setPower(0.75);
    }
    public void reverse() {
        intakeMotor.setPower(-0.75);
    }
    public void stop() {
        intakeMotor.setPower(0.0);
    }
}