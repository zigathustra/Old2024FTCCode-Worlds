package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutoBot extends Bot {
    private final AutoDrivetrain drivetrain;
    public AutoBot(HardwareMap hardwareMap, LinearOpMode opMode, Pose2d startPose, boolean loggingOn)
    {
        super(hardwareMap, opMode, loggingOn);
        drivetrain = new AutoDrivetrain(hardwareMap, telemetry, startPose, loggingOn);
    }

    public void placePurplePixel()
    {

    }

    public AutoDrivetrain drivetrain()
    {
        return drivetrain;
    }
}