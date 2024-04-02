package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.common.enums.Alliance;
import org.firstinspires.ftc.teamcode.common.enums.ParkPosition;
import org.firstinspires.ftc.teamcode.common.enums.StartPosition;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class FarBoard extends org.firstinspires.ftc.teamcode.auto.AutoMaster {

    protected FarBoard(Alliance alliance)
    {
        super(alliance, StartPosition.FAR, ParkPosition.NONE);
    }

    protected void park(double boardDirection, int targetAprilTagNumber, double parkDirection)
    {
    }
}