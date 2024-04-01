package org.firstinspires.ftc.teamcode.opmode.common;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.TeleopBot;

@Config
@TeleOp(name = "TeleOpNormal", group = "Linear OpMode")

public class TeleOpNormal extends LinearOpMode {

    private TeleopBot bot;

    @Override
    public void runOpMode() {

        double driveAxial = 0.0;
        double driveStrafe = 0.0;
        double driveYaw = 0.0;
        double leftTrigger = 0.0;
        double rightTrigger = 0.0;

        bot = new TeleopBot(hardwareMap, telemetry);
        waitForStart();

        while (opModeIsActive() && !gamepad1.ps) {

            if (gamepad1.dpad_up) {
                bot.creepDirection(1.0, 0.0, 0.0);
            } else if (gamepad1.dpad_down) {
                bot.creepDirection(-1.0, 0.0, 0.0);
            } else if (gamepad1.dpad_left) {
                bot.creepDirection(0.0, 1.0, 0.0);
            } else if (gamepad1.dpad_right) {
                bot.creepDirection(0.0, -1.0, 0.0);
            } else {
                driveAxial = gamepad1.left_stick_y;
                driveStrafe = gamepad1.left_stick_x;
                driveYaw = gamepad1.right_stick_x;
                if ((Math.abs(driveAxial) < 0.2) && (Math.abs(driveStrafe) < 0.2) && (Math.abs(driveYaw) < 0.2)) {
                    bot.stopDrive();
                } else
                    bot.moveDirection(-driveAxial, -driveStrafe, -driveYaw);
            }

            leftTrigger = gamepad1.left_trigger;
            rightTrigger = gamepad1.right_trigger;
            if (leftTrigger > 0.3) {
                bot.liftDown(leftTrigger);
            } else if (rightTrigger > 0.3) {
                bot.liftUp(rightTrigger);
            } else {
                bot.liftStop();
            }

            if (gamepad1.x) {
                bot.dropPixel();
            } else if (gamepad1.a) {
                bot.loadPixel();
            }

            if (gamepad1.b) {
                bot.dropperDeploy();
            } else if (gamepad1.y) {
                bot.dropperRetract();
            }

            if (gamepad1.triangle) {
                bot.intakeDeploy();
            } else if (gamepad1.x) {
                bot.intakeRetract();
            }

            if(gamepad1.start){
                bot.launcherRelease();
            } else if(gamepad1.share) {
                bot.launcherLock();
            }
            bot.update();
        }
    }
}
