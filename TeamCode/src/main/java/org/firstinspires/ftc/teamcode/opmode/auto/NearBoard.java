package org.firstinspires.ftc.teamcode.opmode.auto;

import org.firstinspires.ftc.teamcode.common.enums.Alliance;
import org.firstinspires.ftc.teamcode.common.enums.ParkPosition;
import org.firstinspires.ftc.teamcode.common.enums.StartPosition;

public class NearBoard extends AutoMaster {

    protected NearBoard(Alliance alliance)
    {
        super(alliance, StartPosition.NEAR, ParkPosition.NONE);
    }

    protected void park(double boardDirection, int targetAprilTagNumber, double parkDirection)
    {
    }
}