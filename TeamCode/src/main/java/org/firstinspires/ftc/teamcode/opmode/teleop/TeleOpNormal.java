package org.firstinspires.ftc.teamcode.opmode.common;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.TeleopBot;

@Config
@TeleOp(name = "TeleOpNormal", group = "Linear OpMode")

public class TeleOpNormal extends LinearOpMode {

    private TeleopBot robot;

    @Override
    public void runOpMode() {

        double driveAxial = 0.0;
        double driveStrafe = 0.0;
        double driveYaw = 0.0;

        robot = new TeleopBot(hardwareMap, telemetry);
        waitForStart();

        while (opModeIsActive() && !gamepad1.ps) {

            if (gamepad1.dpad_up) {
                robot.creepDirection(1.0, 0.0, 0.0);
            } else if (gamepad1.dpad_down) {
                robot.creepDirection(-1.0, 0.0, 0.0);
            } else if (gamepad1.dpad_left) {
                robot.creepDirection(0.0, 1.0, 0.0);
            } else if (gamepad1.dpad_right) {
                robot.creepDirection(0.0, -1.0, 0.0);
            } else {
                driveAxial = gamepad1.left_stick_y;
                driveStrafe = gamepad1.left_stick_x;
                driveYaw = gamepad1.right_stick_x;
                if ((Math.abs(driveAxial) < 0.2) && (Math.abs(driveStrafe) < 0.2) && (Math.abs(driveYaw) < 0.2)) {
                    robot.stop();
                } else
                    robot.moveDirection(-driveAxial, -driveStrafe, -driveYaw);
            }

            robot.update();
        }
    }
}
