package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.TeleopBot;

@Config
@TeleOp(name = "IntakeTest", group = "Test")

public class IntakeTest extends LinearOpMode {

    private TeleopBot bot;

    public static boolean loggingOn = false;

    @Override
    public void runOpMode() {

        bot = new TeleopBot(hardwareMap, this, loggingOn);
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.right_bumper) {
                bot.load();
            } else if (gamepad1.left_bumper) {
                bot.stopLoad();
            }

            if(gamepad1.start){
                bot.launcherUnlock();
            } else if(gamepad1.share) {
                bot.launcherLock();
            }
            bot.update();
        }
    }
}
