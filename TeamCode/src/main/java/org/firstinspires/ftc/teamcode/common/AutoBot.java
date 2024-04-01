package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutoBot extends Bot {
    private AutoDrivetrain drivetrain = null;
    public AutoBot(HardwareMap hardwareMap, Telemetry telemetry)
    {
        super(hardwareMap, telemetry);
    }
}