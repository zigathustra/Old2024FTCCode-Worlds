package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.config.Constants;

public class Intake extends Component {
    private final Telemetry telemetry;
    private final DcMotorEx intakeMotor;
    private final Servo intakeServo;

    private final double upPos = 0.85;
    private final double downPos = 0.35;
    private final double power = 0.75;


    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        up();
        stop();
    }

    public void deploy()
    {
        down();
        forward();
    }

    public void retract()
    {
        stop();
        up();
    }
    public void load()
    {

    }
    private void up()
    {
        intakeServo.setPosition(upPos);
    }
    private void down()
    {
        intakeServo.setPosition(downPos);
    }

    private void forward()
    {
        intakeMotor.setPower(power);
    }
    private void reverse()
    {
        intakeMotor.setPower(-power);
    }
    private void stop()
    {
        intakeMotor.setPower(0.0);
    }

    public void update()
    {

    }
}