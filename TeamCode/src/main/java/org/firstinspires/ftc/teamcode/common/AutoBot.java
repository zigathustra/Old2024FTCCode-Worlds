package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutoBot extends Bot {
    private final AutoDrivetrain drivetrain;
    public AutoBot(HardwareMap hardwareMap, Telemetry telemetry, Pose2d startPose)
    {
        super(hardwareMap, telemetry);
        drivetrain = new AutoDrivetrain(hardwareMap, startPose);
    }

    public void placePurplePixel()
    {
        dropperDeploy();
        lift.goToPurplePlacementPosition();
        dropPixel();
    }

    public AutoDrivetrain drivetrain()
    {
        return drivetrain;
    }
}