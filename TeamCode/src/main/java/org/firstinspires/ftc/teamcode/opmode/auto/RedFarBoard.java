package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.enums.Alliance;
import org.firstinspires.ftc.teamcode.common.enums.Alliance;

import org.firstinspires.ftc.teamcode.auto.FarBoard;

@Autonomous(name = "\uD83D\uDD25RedFarBoard", group = "RedFar")
public class RedFarBoard extends FarBoard {
    public RedFarBoard() {
        super(Alliance.RED);
    }
}