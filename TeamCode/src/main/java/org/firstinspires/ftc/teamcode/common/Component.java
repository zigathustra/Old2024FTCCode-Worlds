package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Component {

    protected Telemetry telemetry;
    protected boolean loggingOn = false;

    public Component(Telemetry telemetry, boolean loggingOn)
    {
        this.telemetry = telemetry;
        this.loggingOn = loggingOn;
    }

    public abstract void update();

}
