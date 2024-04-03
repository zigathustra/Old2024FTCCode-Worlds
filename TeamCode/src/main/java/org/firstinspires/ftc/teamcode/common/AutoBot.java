package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutoBot extends Bot {
    private final AutoDrivetrain drivetrain;
    public AutoBot(HardwareMap hardwareMap, Telemetry telemetry, Pose2d startPose, boolean loggingOn)
    {
        super(hardwareMap, telemetry, loggingOn);
        drivetrain = new AutoDrivetrain(hardwareMap, telemetry, startPose, loggingOn);
    }

    public void placePurplePixel()
    {
        dropperDeploy();
//        lift.goToPurplePlacementPosition();
        dropPixel();
    }

    public AutoDrivetrain drivetrain()
    {
        return drivetrain;
    }
}