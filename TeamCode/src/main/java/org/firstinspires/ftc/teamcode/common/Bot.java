package org.firstinspires.ftc.teamcode.common;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.common.config.Constants;

public class Bot {
    // Attributes for hardware
    protected DriveTrain driveTrain = null;
    protected Lift lift = null;
    protected Servo wrist = null;
    protected Servo grabber = null;
    protected Servo launcher = null;
    protected Servo pokey = null;
    protected Rev2mDistanceSensor distanceSensor = null;
    protected TouchSensor touchSensor = null;
    protected LinearOpMode opMode = null;
    protected double maxSpeed = Constants.maxNormalSpeed; // Default speed. Reassigned in the constructor.

    public Bot(LinearOpMode opMode, double maxSpeed) {
        this.opMode = opMode;
        this.maxSpeed = maxSpeed;

//        opMode.telemetry.addData("drivetrain ", 1);
//        opMode.telemetry.update();
//        opMode.sleep (1000);

        driveTrain = new DriveTrain(opMode, maxSpeed);

//        opMode.telemetry.addData("lift ", 1);
//        opMode.telemetry.update();
//        opMode.sleep (1000);

        lift = new Lift(opMode);

//        opMode.telemetry.addData("wrist ", 1);
//        opMode.telemetry.update();
//        opMode.sleep (1000);

        wrist = opMode.hardwareMap.get(Servo.class, "wrist");
        wrist.setPosition(Constants.wristDownPosition);

//        opMode.telemetry.addData("grabber ", 1);
//        opMode.telemetry.update();
//        opMode.sleep (1000);
        grabber = opMode.hardwareMap.get(Servo.class, "grabber");
        grabber.setPosition(Constants.grabberClosedPosition);

        launcher = opMode.hardwareMap.get(Servo.class, "launcher");
        launcher.setPosition(Constants.launcherLockedPosition);

        pokey = opMode.hardwareMap.get(Servo.class, "pokey");
        pokey.setPosition(Constants.pokeyDownPosition);
//        opMode.telemetry.addData("d_sensor ", 1);
//        opMode.telemetry.update();
//        opMode.sleep (1000);
        distanceSensor = opMode.hardwareMap.get(Rev2mDistanceSensor.class, "distance_sensor");

        touchSensor = opMode.hardwareMap.get(TouchSensor.class, "touch_sensor");
    }

    public void setRunWithoutEncoders() {
        driveTrain.setRunWithoutEncoders();
    }

    public double getHeading()
    {
        return driveTrain.getHeading();
    };

    public void setToFastSpeed()
    {
        driveTrain.setToFastSpeed();
    }
    public void setToSlowedSpeed()
    {
        driveTrain.setToSlowedSpeed();
    }
    // Turn to a specified heading in degrees
    // 0 is straight ahead
    // > 0 is CCW
    // < 0 is CW
    public void turnToHeading(double heading) {
        driveTrain.turnToHeading(heading);
    }

    // Turn a specified distance in degrees
    public void turnForDistance(double distance) {
        driveTrain.turnForDistance(distance);
    }

    public void moveDirection(double axial, double strafe, double yaw) {
        driveTrain.moveDirection(axial, strafe, yaw);
    }

    public void moveDirectionNoEnc(double axial, double strafe, double yaw) {
        driveTrain.moveDirection(axial, strafe, yaw);
    }

    public void creepDirection(double axial, double strafe, double yaw) {
        driveTrain.creepDirection(axial, strafe, yaw);
    }

    public void creepStraightForDistance(double distance) {
        driveTrain.creepStraightForDistance(distance);
    }

    public void creepUntilContact() {
        creepDirection(1,0,0);
        while (!touchSensor.isPressed())
        {
        }
        stopDrive();
    }

    // Move straight for a specified distance in inches
    public void moveStraightForDistance(double distance) {
        driveTrain.moveStraightForDistance(distance);
    }

    public void strafeForDistance(double distance) {driveTrain.strafeForDistance(distance);
    }

    public void stopDrive() {
        driveTrain.moveDirection(0, 0, 0);
    }

    public void liftUp(double speed) {
        lift.liftUp(speed);
    }

    public void liftDown(double speed) {
        lift.liftDown(speed);
    }

    public void liftStop() {lift.stop();
    }

    public void liftStopAtPosition(int position) {
        lift.stopAtPosition(position);
    }

    public void liftZero(){lift.liftZero();}

    public void wristUp() {
        wrist.setPosition(Constants.wristUpPosition);
    }

    public void wristMiddle() {
        wrist.setPosition(Constants.wristMiddlePosition);
    }
    public void wristDown() {
        wrist.setPosition(Constants.wristDownPosition);
    }
    public void wristToPosition(double position) {
        wrist.setPosition(position);
    }
    public void grabberOpen() {
        grabber.setPosition(Constants.grabberOpenPosition);
    }

    public void grabberClose() {
        grabber.setPosition(Constants.grabberClosedPosition);
    }

    public void launcherLocked(){ launcher.setPosition(Constants.launcherLockedPosition);}

    public void launcherUnlocked(){ launcher.setPosition(Constants.launcherUnlockedPosition);}
    public void pokeyDown(){ pokey.setPosition(Constants.pokeyDownPosition);}
    public void pokeyUp(){ pokey.setPosition(Constants.pokeyUpPosition);}

    public double getDistance() {
        double distance;
        opMode.sleep(50);
        distance = distanceSensor.getDistance(DistanceUnit.INCH);
        opMode.sleep(50);
        return (distance);
    }
}


