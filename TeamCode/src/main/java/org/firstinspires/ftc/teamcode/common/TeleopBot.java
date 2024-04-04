package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TeleopBot extends Bot {
    private TeleopDrivetrain drivetrain = null;
    public TeleopBot(HardwareMap hardwareMap, LinearOpMode opMode, boolean loggingOn)
    {
        super(hardwareMap, opMode, loggingOn);
        drivetrain = new TeleopDrivetrain(hardwareMap, telemetry, loggingOn);
    }

    public void creepDirection(double axial, double strafe, double yaw) {
        drivetrain.creepDirection(axial, strafe, yaw);
    }

    public void moveDirection(double axial, double strafe, double yaw) {
        drivetrain.moveDirection(axial, strafe, yaw);
    }
    public void stopDrive() {
        drivetrain.stop();
    }
}
