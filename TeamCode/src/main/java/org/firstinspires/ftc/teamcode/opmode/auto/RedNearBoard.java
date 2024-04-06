package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.enums.Alliance;

@Autonomous(name = "\uD83D\uDD25RedNearBoard", group = "Auto")
public class RedNearBoard extends NearBoard {
    public RedNearBoard() {
        super(Alliance.RED);
    }
}