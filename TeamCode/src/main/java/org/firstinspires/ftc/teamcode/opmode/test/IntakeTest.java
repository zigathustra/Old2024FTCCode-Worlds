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

    @Override
    public void runOpMode() {

        bot = new TeleopBot(hardwareMap, telemetry);
        waitForStart();

        while (opModeIsActive() && !gamepad1.ps) {

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
