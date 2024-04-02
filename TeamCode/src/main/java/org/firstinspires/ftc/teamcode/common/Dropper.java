package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Dropper extends Component {
    private final ColorRangeSensor lowerSensor;
    private final ColorRangeSensor upperSensor;
    private final Servo lowerServo;
    private final Servo upperServo;

    private final double upperServoLockPos = 1;
    private final double upperServoReleasePos = 0.5;

    private final double lowerServoLockPos = 1;
    private final double lowerServoReleasePos = 0.5;

    public Dropper(HardwareMap hardwareMap, Telemetry telemetry)
    {
        lowerSensor = hardwareMap.get(ColorRangeSensor.class, "dropLowerSensor");
        upperSensor = hardwareMap.get(ColorRangeSensor.class, "dropUpperSensor");
        lowerServo = hardwareMap.get(Servo.class, "dropLowerServo");
        upperServo = hardwareMap.get(Servo.class, "dropUpperServo");
    }

    public void dropPixel()
    {
    }

    public void loadPixel()
    {
    }

    public void releaseServos()
    {
        lowerServo.setPosition(lowerServoReleasePos);
        upperServo.setPosition(upperServoReleasePos);
    }

    public void lockServos()
    {
        lowerServo.setPosition(lowerServoLockPos);
        upperServo.setPosition(upperServoLockPos);
    }
    public void update()
    {

    }
}
