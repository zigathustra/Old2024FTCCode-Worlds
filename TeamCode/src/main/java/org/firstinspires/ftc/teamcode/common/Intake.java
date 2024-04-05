package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends Component {
    private final DcMotorEx intakeMotor;
    private final Servo intakeServo;
    private final double upPos = 1.0;
    private final double downPos = 0.15;
    private final double power = 0.00;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry, boolean loggingOn) {
        super(telemetry, loggingOn);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeServo.setDirection(Servo.Direction.FORWARD);
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
    public void up()
    {
        intakeServo.setPosition(upPos);
    }
    public void down()
    {
        intakeServo.setPosition(downPos);
    }

    public void forward()
    {
        intakeMotor.setPower(power);
    }
    public void manualForward(double power)
    {
        intakeMotor.setPower(power);
    }
    private void reverse()
    {
        intakeMotor.setPower(-power);
    }
    public void stop()
    {
        intakeMotor.setPower(0.0);
    }

    public void update()
    {
    }
}