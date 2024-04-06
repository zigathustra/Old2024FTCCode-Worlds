package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.AutoDrivetrain;
import org.firstinspires.ftc.teamcode.odometry.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.odometry.TwoDeadWheelLocalizer;

public final class ManualFeedbackTuner extends LinearOpMode {
    public static double DISTANCE = 64;

    public static boolean loggingOn = true;

    @Override
    public void runOpMode() throws InterruptedException {
        AutoDrivetrain drive = new AutoDrivetrain(hardwareMap, telemetry, new Pose2d(0, 0, 0), loggingOn);
        if (ThreeDeadWheelLocalizer.PARAMS.perpXTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par0YTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par1YTicks == 1) {
            throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
        }
        waitForStart();

        while (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(0, 0, 0))
                            .lineToX(DISTANCE)
                            .lineToX(0)
                            .build());
        }
    }
}
