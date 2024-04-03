package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Bot extends Component {
    protected Telemetry telemetry = null;
    private Intake intake = null;
    protected Lift lift = null;
    private Servo shoulderL = null;
    private Servo shoulderR = null;
    private Servo wrist = null;
    private Dropper dropper = null;
    private Servo launcher = null;
    private boolean dropperDeployed = false;

    private double shoulderRUpPos = 1.0;
    //    private double shoulderRMidPos = 0.5;
    private double shoulderRDownPos = 0.1;
    private double shoulderLUpPos = 0.9;
    //    private double shoulderLMidPos = 0.5;
    private double shoulderLDownPos = 0.0;

    private double wristUpPos = 0.9;
    private double wristDownPos = 0.15;
    private double launcherLockPos = 0.75;
    private double launcherReleasePos = 0.25;
    private boolean loading = false;

    protected boolean loggingOn;


    public Bot(HardwareMap hardwareMap, Telemetry telemetry, boolean loggingOn) {
        super(telemetry, loggingOn);
        // Intake
        intake = new Intake(hardwareMap, telemetry, loggingOn);

        // Lift
        lift = new Lift(hardwareMap, telemetry, loggingOn);

        // Shoulder
        shoulderL = hardwareMap.get(Servo.class, "shoulderL");
        shoulderR = hardwareMap.get(Servo.class, "shoulderR");
        shoulderL.setDirection(Servo.Direction.REVERSE);
        shoulderR.setDirection(Servo.Direction.FORWARD);

        // Wrist
        wrist = hardwareMap.get(Servo.class, "wrist");
        wrist.setDirection(Servo.Direction.REVERSE);

        // Dropper
        dropper = new Dropper(hardwareMap, telemetry, loggingOn);
        dropperDeployed = false;
        loading = false;

        // Launcher
        launcher = hardwareMap.get(Servo.class, "launcher");
        launcher.setDirection(Servo.Direction.FORWARD);
        launcher.setPosition(launcherLockPos);

        dropperDeploy();
    }


    public void dropPixel()
    {
        dropper.dropPixel();
    }

    public void dropperDeploy() {
        lift.setTargetPos(350);
        wrist.setPosition(wristUpPos);
        shoulderL.setPosition(shoulderLUpPos);
        shoulderR.setPosition(shoulderRUpPos);
        dropperDeployed = true;
    }

    public void dropperRetract() {
        lift.setTargetPos(350);
        wrist.setPosition(wristDownPos);
        shoulderL.setPosition(shoulderLDownPos);
        shoulderR.setPosition(shoulderRDownPos);
        dropperDeployed = false;
    }

    public void load() {
        if (!dropperDeployed) {
            dropper.load();
            intake.deploy();
            loading = true;
        }
    }

    public void stopLoad() {
        intake.retract();
        dropper.stopLoad();
        loading = false;
    }

    public void liftUp(double power) {
        if (dropperDeployed) {
            lift.up(power);
        }
    }

    public void liftDown(double power) {
        if (dropperDeployed) {
            lift.down(power);
        }
    }

    public void launcherLock() {
        launcher.setPosition(launcherLockPos);
    }

    public void launcherRelease() {
        launcher.setPosition(launcherReleasePos);
    }

    public void update() {
        lift.update();
        intake.update();
        dropper.update();
        if (dropper.fullyLoaded() && loading)
        {
            loading = false;
            dropper.stopLoad();
        }
    }
}
