package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Lift;
import org.firstinspires.ftc.teamcode.common.TeleopBot;

@Config
@TeleOp(name = "LiftTest", group = "Test")

public class LiftTest extends LinearOpMode {

    private Lift lift;
    @Override
    public void runOpMode() {
        double leftTrigger = 0.0;
        double rightTrigger = 0.0;

        lift = new Lift(hardwareMap, telemetry);
        waitForStart();

        while (opModeIsActive()) {

            leftTrigger = gamepad1.left_trigger;
            rightTrigger = gamepad1.right_trigger;
            if (leftTrigger > 0.3) {
                lift.down(leftTrigger);
            } else if (rightTrigger > 0.3) {
                lift.up(rightTrigger);
            } else {
//                lift.liftStop();
            }
            lift.update();;
        }
    }
}
