package org.firstinspires.ftc.teamcode.opmode.common;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Intake;
import org.firstinspires.ftc.teamcode.common.TeleOpDriveTrain;
import org.firstinspires.ftc.teamcode.common.Lift;
@Config
@TeleOp(name = "TeleOpNormal", group = "Linear OpMode")

public class TeleOpNormal extends LinearOpMode {

    private TeleOpDriveTrain driveTrain = null;
    private Intake intake = null;
    private Lift lift = null;

    @Override
    public void runOpMode() {

        double driveAxial = 0.0;
        double driveStrafe = 0.0;
        double driveYaw = 0.0;
        double leftTrigger = 0.0;
        double rightTrigger = 0.0;
        boolean liftStopped = true;
        driveTrain = new TeleOpDriveTrain(this);
        intake = new Intake(this);
        lift = new Lift(this);
        waitForStart();

        while (opModeIsActive()&&!gamepad1.ps) {
            // Drivetrain
            if (gamepad1.dpad_up) {
                driveTrain.creepDirection(1.0, 0.0, 0.0);
            } else if (gamepad1.dpad_down) {
                driveTrain.creepDirection(-1.0, 0.0, 0.0);
            } else if (gamepad1.dpad_left) {
                driveTrain.creepDirection(0.0, 1.0, 0.0);
            } else if (gamepad1.dpad_right) {
                driveTrain.creepDirection(0.0, -1.0, 0.0);
            } else {
                driveAxial = gamepad1.left_stick_y;
                driveStrafe = gamepad1.left_stick_x;
                driveYaw = gamepad1.right_stick_x;
                if ((Math.abs(driveAxial) < 0.2) && (Math.abs(driveStrafe) < 0.2) && (Math.abs(driveYaw) < 0.2)) {
                    driveTrain.stop();
                } else
                    driveTrain.moveDirection(-driveAxial, -driveStrafe, -driveYaw);
            }

            // Intake
            if(gamepad1.right_bumper){
                intake.lower();
                intake.forward();
            }else if(gamepad1.left_bumper){
                intake.raise();
                intake.stop();
            }

            // Lift
            leftTrigger = gamepad1.left_trigger;
            rightTrigger = gamepad1.right_trigger;
            if (leftTrigger > 0.3) {
                lift.liftDown(0.35);
                liftStopped = false;
            } else if (rightTrigger > 0.3) {
                lift.liftUp(0.35);
                liftStopped = false;
            } else if (!liftStopped){
                lift.stop();
                liftStopped = true;
            }

            // Shoulder

            // Dropper

            // Launcher

/*
            if (gamepad1.x) {
                driveTrain.grabberClose();
            } else if (gamepad1.a) {
                driveTrain.grabberOpen();
            }

            if (gamepad1.b) {
                driveTrain.wristUp();
            } else if (gamepad1.y) {
                driveTrain.wristMiddle();
            }
            if (gamepad1.share && gamepad1.start) {
                driveTrain.liftZero();
            }
            if(gamepad1.start){
                driveTrain.launcherUnlocked();
            } else if(gamepad1.share) {
                driveTrain.launcherLocked();
            }
            if(gamepad1.right_bumper){
                driveTrain.pokeyUp();
            }else if(gamepad1.left_bumper){
                driveTrain.pokeyDown();
            }

 */
        }
    }
}
