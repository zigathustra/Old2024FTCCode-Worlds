package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Bot extends Component {
    protected Telemetry telemetry = null;
    private Intake intake = null;
    private Lift lift = null;
    private Servo shoulderL = null;
    private Servo shoulderR = null;
    private Servo wrist = null;
    private Dropper dropper = null;
    private Servo launcher = null;

    private double shoulderUpPos = 0.75;
    private double shouldMidPos = 0.5;
    private double shoulderDownPos = 0.25;
    private double wristUpPos = 0.75;
    private double wristMidPos = 0.5;
    private double wristDownPos = 0.25;
    private double launcherLockPos =  0.75;
    private double launcherReleasePos = 0.25;


    public Bot(HardwareMap hardwareMap, Telemetry telemetry)
    {
        this.telemetry = telemetry;

        // Intake
        intake = new Intake(hardwareMap, telemetry);

        // Lift
        lift = new Lift(hardwareMap, telemetry);

        // Shoulder
        shoulderL = hardwareMap.get(Servo.class,"shoulderL");
        shoulderR = hardwareMap.get(Servo.class,"shoulderR");
        shoulderL.setDirection(Servo.Direction.REVERSE);
        shoulderR.setDirection(Servo.Direction.FORWARD);
        shoulderL.setPosition(shoulderDownPos);
        shoulderR.setPosition(shoulderDownPos);

        // Wrist
        wrist = hardwareMap.get(Servo.class,"wrist");
        wrist.setDirection(Servo.Direction.FORWARD);
        wrist.setPosition(wristDownPos);

        // Dropper
        dropper = new Dropper(hardwareMap, telemetry);

        // Launcher
        launcher = hardwareMap.get(Servo.class,"launcher");
        launcher.setDirection(Servo.Direction.FORWARD);
        launcher.setPosition(launcherLockPos);

    }

    public void loadPixel()
    {

    }

    public void dropPixel()
    {
        dropper.dropPixel();
    }
    public void dropperDeploy()
    {

    }

    public void dropperRetract()
    {

    }

    public void intakeDeploy()
    {
        intake.deploy();
    }

    public void intakeRetract()
    {
        intake.retract();
    }
    public void liftUp(double speed)
    {
        lift.up(speed);
    }

    public void liftDown(double speed)
    {
        lift.down(speed);
    }

    public void liftStop()
    {
    }

    public void launcherLock()
    {
        launcher.setPosition(launcherLockPos);
    }
    public void launcherRelease()
    {
        launcher.setPosition(launcherReleasePos);
    }

    public void update()
    {
        lift.update();
        intake.update();
        dropper.update();
    }
}
