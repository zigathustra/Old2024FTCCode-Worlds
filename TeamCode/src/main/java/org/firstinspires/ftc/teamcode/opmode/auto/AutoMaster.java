package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import org.firstinspires.ftc.teamcode.common.AutoBot;
import org.firstinspires.ftc.teamcode.common.Bot;
import org.firstinspires.ftc.teamcode.common.enums.Alliance;
import org.firstinspires.ftc.teamcode.common.enums.ParkPosition;
import org.firstinspires.ftc.teamcode.common.enums.PropDirection;
import org.firstinspires.ftc.teamcode.common.enums.StartPosition;
import org.firstinspires.ftc.teamcode.common.vision.PropPipeline;
import org.firstinspires.ftc.teamcode.common.vision.VisionSensor;
import org.firstinspires.ftc.teamcode.opmode.auto.AutoConstants;
import org.firstinspires.ftc.teamcode.opmode.auto.AutoOpMode;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public abstract class AutoMaster extends LinearOpMode {
    protected Alliance alliance;
    protected StartPosition startPosition;
    protected ParkPosition parkPosition;
    int riggingDirection;
    int boardDirection;
    int parkDirection;

    public static boolean loggingOn = false;

    Action selectedTrajectory;
    int targetAprilTagNumber;
    PropPipeline propProcessor = null;
    AprilTagProcessor aprilTagProcessor;
    PropDirection propDirection = null;
    ElapsedTime timer = new ElapsedTime();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

    protected AutoBot bot;
    protected VisionSensor visionSensor;

    Action leftSpikeTrajectory;
    Action centerSpikeTrajectory;
    Action rightSpikeTrajectory;

    // Start Pose Constants
    protected final Pose2d blueFarStartPose = new Pose2d(-43, 62.00, Math.toRadians(-90));
    protected final Pose2d blueNearStartPose = new Pose2d(15.85, 62.00, Math.toRadians(-90));
    protected final Pose2d redFarStartPose = new Pose2d(-43, -62.00, Math.toRadians(90));
    protected final Pose2d redNearStartPose = new Pose2d(15.85, -62.00, Math.toRadians(90));
    protected AutoConstants autoConstants = new AutoConstants();

    protected AutoMaster(Alliance alliance, StartPosition startPosition, ParkPosition parkPosition) {
        this.alliance = alliance;
        this.startPosition = startPosition;
        this.parkPosition = parkPosition;
    }

    @Override
    public void runOpMode() {
        bot = new AutoBot(hardwareMap, telemetry, determineStartPose(), loggingOn);

        visionSensor = new VisionSensor(this, alliance);

        riggingDirection = determineRiggingDirection();

        boardDirection = determineBoardDirection(riggingDirection);

        parkDirection = determineParkDirection(parkPosition, boardDirection);

        visionSensor.goToPropDetectionMode();

        bot.dropperRetract();
        bot.stopLoad();
        sleep(500);

        leftSpikeTrajectory = bot.drivetrain().actionBuilder(bot.drivetrain().pose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(3)
                .build();

        while (!isStarted() && !isStopRequested()) {
            propDirection = visionSensor.getPropDirection();

            telemetry.addData("Prop Position: ", propDirection);
            telemetry.update();

            sleep(50);
        }
        if (!isStopRequested()) {
            visionSensor.goToNoSensingMode();
            selectedTrajectory = determineSelectedTrajectory();
            timer.reset();

            Actions.runBlocking(
                    new SequentialAction(
                            selectedTrajectory
                    )
            );

            bot.placePurplePixel();
        }
    }

    /*            setToLowCruisingPosition();

                // Place pixel on correct spike mark and return to escape position
                // Use propDirection determined using webcam during init
                placePropPixel(propDirection, riggingDirection);

                roughTravelToBoard(boardDirection, riggingDirection);

                visionSensor.goToAprilTagDetectionMode();

                targetAprilTagNumber = getTargetAprilTagNumber(alliance, propDirection);

                roughAlignToAprilTag(boardDirection, targetAprilTagNumber, startPosition);
    //        if (timer.time() <= orientMaxTime()) {
                // Correct strafe to directly face the target April Tag
                autoOrientToAprilTag(visionSensor, targetAprilTagNumber, boardDirection);
    //        }

    //        if (timer.time() <= placeMaxTime()) {            // Correct strafe to directly face the target April Tag
                placePixelOnBoard();
    //        }

    //        if (timer.time() <= parkMaxTime()) {
                park(boardDirection, targetAprilTagNumber, parkDirection);
    //        }
    //        telemetry.addData("finish park Time: ",timer.time());
    //        telemetry.update();
    //        sleep(1000);
                // Lower lift, lower wrist, open grabber

                setToTeleopStartingPosition();
            }
        }

    */
    protected Action determineSelectedTrajectory() {
        if (propDirection == PropDirection.LEFT) {
            return leftSpikeTrajectory;
        } else if (propDirection == PropDirection.RIGHT) {
            return rightSpikeTrajectory;
        } else {
            return centerSpikeTrajectory;
        }
    }

    protected int determineRiggingDirection() {
        if (((startPosition == StartPosition.FAR) && (alliance == Alliance.BLUE)) ||
                ((startPosition == StartPosition.NEAR) && (alliance == Alliance.RED))) {
            return (-1);
        } else {
            return (1);
        }
    }

    protected int determineBoardDirection(int riggingDirection) {
        if (startPosition == StartPosition.FAR) {
            return (riggingDirection);
        } else {
            return (-riggingDirection);
        }
    }

    protected int determineParkDirection(ParkPosition parkPosition, int boardDirection) {
        if (parkPosition == ParkPosition.CORNER) {
            return (boardDirection);
        } else if (parkPosition == ParkPosition.CENTER) {
            return (-boardDirection);
        } else {
            return (0);
        }
    }

    protected Pose2d determineStartPose() {
        if (alliance == Alliance.RED) {
            if (startPosition == StartPosition.FAR) {
                return (redFarStartPose);
            } else {
                return (redNearStartPose);
            }
        } else if (startPosition == StartPosition.FAR) {
            return blueFarStartPose;
        } else {
            return blueNearStartPose;
        }
    }

    /*
    protected void placePropPixel(PropDirection propDirection, int riggingDirection) {
        double setupDistance = 0;
        double placementDistance = Constants.pdCenterPlacementDistance;
        double heading = Constants.pdCenterHeading;
        double correctionDistance = 0;

        if (propDirection == PropDirection.LEFT) {
            placementDistance = Constants.pdLeftPlacementDistance;
            heading = Constants.pdLeftHeading;
            correctionDistance = 4;
            setupDistance = 15;
        } else if (propDirection == PropDirection.RIGHT) {
            placementDistance = Constants.pdRightPlacementDistance;
            heading = Constants.pdRightHeading;
            setupDistance = 15;
        }

//        telemetry.addData("propDirection: ", propDirection);
//        telemetry.addData("Heading: ", heading);
//        telemetry.addData("Distance: ", distance);
//        telemetry.update();
//        sleep(5000);

        bot.moveStraightForDistance(setupDistance);
        bot.turnToHeading(heading);
        bot.moveStraightForDistance(placementDistance);
        bot.moveStraightForDistance(-placementDistance - correctionDistance);
        bot.turnToHeading(0);
        bot.strafeForDistance(-correctionDistance);
        bot.moveStraightForDistance(-setupDistance);
        bot.strafeForDistance(-(riggingDirection * Constants.pdEscapeStrafeDistance));
    }

    protected void roughTravelToBoard(int boardPosition, int riggingDirection) {
        bot.turnToHeading(riggingDirection * 90);
    }

     */
    public int getTargetAprilTagNumber(Alliance alliance, PropDirection propDirection) {
        int aprilTagNumber = 5;

        if (alliance == Alliance.RED) {
            aprilTagNumber = 4;
            if (propDirection == PropDirection.CENTER) {
                aprilTagNumber = 5;
            } else if (propDirection == PropDirection.RIGHT) {
                aprilTagNumber = 6;
            }
        } else {
            aprilTagNumber = 1;
            if (propDirection == PropDirection.CENTER) {
                aprilTagNumber = 2;
            } else if (propDirection == PropDirection.RIGHT) {
                aprilTagNumber = 3;
            }
        }
        return (aprilTagNumber);
    }

    /*
    protected void roughAlignToAprilTag(int boardDirection,
                                        int targetAprilTagNumber, StartPosition startPosition) {
        double strafeVector = 0;
        double chassisWidth = 2 * Constants.sensorToDrivetrainMiddle;
        if (boardDirection == -1) {
            if (startPosition == StartPosition.FAR) {
                strafeVector = (targetAprilTagNumber - 4 - 1.2) * Constants.distanceBetweenAprilTags;
            } else {
                strafeVector = chassisWidth + (targetAprilTagNumber - .6) * Constants.distanceBetweenAprilTags;
            }
        } else {
            if (startPosition == StartPosition.FAR) {
                strafeVector = chassisWidth + (targetAprilTagNumber - 4 + 2) * Constants.distanceBetweenAprilTags;
            } else {
                strafeVector = -2 - (6 - targetAprilTagNumber) * Constants.distanceBetweenAprilTags;
            }
        }
        bot.strafeForDistance(strafeVector);
    }

    protected void autoOrientToAprilTag(VisionSensor visionSensor, int targetTagNumber, int boardDirection) {

        AprilTagDetection targetTag;
        boolean targetFound = false;
        double strafeError;
        double yawError;
        double strafePower = 1;
        double yawPower = 1;
        double minStrafePower = .01;
        double minYawPower = .01;
        double currentHeading = 0;
        double startHeading = bot.getHeading();
        double maxScanHeading = startHeading + 15;
        double minScanHeading = startHeading - 15;
        ElapsedTime scanTimer = new ElapsedTime();
        double scanDuration = 4;
        double yawSearchPower = .15;
        double scanDirection = 1;

        scanTimer.reset();

        while (((Math.abs(strafePower) > minStrafePower) || (Math.abs(yawPower) > minYawPower))
                && (scanTimer.time() < scanDuration)) {
            targetFound = false;
            targetTag = null;

//            telemetry.addData("strafePower: ",strafePower);
//            telemetry.addData("yawPower: ",yawPower);
//            telemetry.update();
//            sleep(500);

            // Search through detected tags to find the target tag
            List<AprilTagDetection> currentDetections = visionSensor.getAprilTagDetections();
            for (AprilTagDetection detection : currentDetections) {
//                telemetry.addData("Iter Detections: ", targetFound);
//                telemetry.update();
//                sleep(100);

                // Look to see if we have size info on this tag
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag
                    if (detection.id == targetTagNumber) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        targetTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("Found", "ID %d (%s)", targetTag.id, targetTag.metadata.name);
                telemetry.addData("Range", "%5.1f inches", targetTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", targetTag.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f degrees", targetTag.ftcPose.yaw);

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                strafeError = targetTag.ftcPose.bearing;
                yawError = targetTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                strafePower = Range.clip(-yawError * Constants.atStrafeGain, -Constants.atMaxStrafe, Constants.atMaxStrafe);
                yawPower = Range.clip(strafeError * Constants.atYawGain, -Constants.atMaxYaw, Constants.atMaxYaw);

                telemetry.addData("Auto", "Strafe %5.2f", strafePower);
                telemetry.addData("Auto", "Yaw %5.2f", yawPower);
            } else {
                strafePower = 0;
                currentHeading = bot.getHeading();
                if (currentHeading <= minScanHeading) {
                    scanDirection = 1;
                }
                if (currentHeading >= maxScanHeading) {
                    scanDirection = -1;
                }
                yawPower = scanDirection * yawSearchPower;
            }
            telemetry.update();
            // Apply desired axes motions to the drivetrain.
            bot.moveDirection(0, strafePower, yawPower);
            sleep(10);
        }
        if (!targetFound) {
            bot.turnToHeading(boardDirection * -90);
        }
    }
*/
    protected void placePixelOnBoard() {
 /*       bot.moveStraightForDistance(Constants.boardApproachDistance);
        bot.strafeForDistance(-Constants.sensorToDrivetrainMiddle);
        bot.liftStopAtPosition(Constants.liftAutoBoardProbePosition);
        bot.wristDown();
        bot.creepUntilContact();
        bot.creepStraightForDistance(-Constants.boardOffsetDistance);
        bot.wristUp();
//        bot.wristToPosition(Constants.wristUpPosition);
        bot.liftStopAtPosition(Constants.liftAutoBoardPlacementPosition);
        sleep(250);
        bot.creepStraightForDistance(Constants.boardOffsetDistance + 4.5);
        bot.grabberOpen();
        sleep(250);
        bot.moveStraightForDistance(-Constants.boardEscapeDistance);

  */
    }

    protected void park(double boardDirection, int targetAprilTagNumber, double parkDirection) {
        double strafeVector = 0;
        int adjustedTagNumber = targetAprilTagNumber;
        if (targetAprilTagNumber > 3) {
            adjustedTagNumber = adjustedTagNumber - 3;
        }
 /*       strafeVector = 2.5 * Constants.sensorToDrivetrainMiddle;

        if (parkDirection > 0) {
            strafeVector = strafeVector + (4.5 - adjustedTagNumber) * Constants.distanceBetweenAprilTags;
        } else {
            strafeVector = -strafeVector - 3 - adjustedTagNumber * Constants.distanceBetweenAprilTags;
        }

        bot.turnToHeading(boardDirection * 90);
        bot.strafeForDistance(-strafeVector);
        bot.moveStraightForDistance(-14);
    }



    protected void setToTeleopStartingPosition() {
        bot.grabberClose();
        bot.liftStopAtPosition(Constants.liftAutoHighCruisingPosition);
        bot.wristMiddle();
        sleep(250);
        bot.liftStopAtPosition(0);
        sleep(2500);
    }

  */
    }
}