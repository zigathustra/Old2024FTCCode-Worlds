package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Lift;

@Config
@TeleOp(name = "LiftTestPIDTune", group = "Test")

public class LiftTestPIDTune extends LinearOpMode {
    private Lift lift;
    @Override
    public void runOpMode() {
        double leftTrigger = 0.0;
        double rightTrigger = 0.0;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        lift = new Lift(hardwareMap, telemetry, true);
        waitForStart();

        while (opModeIsActive()) {

            leftTrigger = gamepad1.left_trigger;
            rightTrigger = gamepad1.right_trigger;
            if (gamepad1.right_bumper)
            {
                lift.setTargetPos(1000);
            } else if (gamepad1.left_bumper)
            {
                lift.setTargetPos(500);
            }

            if (rightTrigger > 0.3)
            {
                lift.up(rightTrigger);           }
            else if (leftTrigger > 0.3)
            {
                lift.down(leftTrigger);
            }

            lift.update();
        }
    }
}
