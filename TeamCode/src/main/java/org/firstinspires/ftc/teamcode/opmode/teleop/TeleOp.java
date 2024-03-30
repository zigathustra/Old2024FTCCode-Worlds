package org.firstinspires.ftc.teamcode.opmode.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.TeleOpDriveTrain;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleoOp", group = "Linear OpMode")
public class TeleOp extends LinearOpMode {

    private TeleOpDriveTrain driveTrain = null;

    @Override
    public void runOpMode() {

        double driveAxial = 0.0;
        double driveStrafe = 0.0;
        double driveYaw = 0.0;
        double leftTrigger = 0.0;
        double rightTrigger = 0.0;
        driveTrain = new TeleOpDriveTrain(this);
        waitForStart();

        while (opModeIsActive()&&!gamepad1.ps) {

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

/*            leftTrigger = gamepad1.left_trigger;
            rightTrigger = gamepad1.right_trigger;
            if (leftTrigger > 0.3) {
                driveTrain.liftDown(leftTrigger);
            } else if (rightTrigger > 0.3) {
                driveTrain.liftUp(rightTrigger);
            } else {
                driveTrain.liftStop();
            }

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
