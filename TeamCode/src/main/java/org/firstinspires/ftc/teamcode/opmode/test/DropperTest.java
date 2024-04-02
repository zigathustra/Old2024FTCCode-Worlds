package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.TeleopBot;

@Config
@TeleOp(name = "DropperTest", group = "Test")

public class DropperTest extends LinearOpMode {

    private TeleopBot bot;

    @Override
    public void runOpMode() {

        bot = new TeleopBot(hardwareMap, telemetry);
        waitForStart();

        while (opModeIsActive() && !gamepad1.ps) {


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

            bot.update();
        }
    }
}
