package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Bot extends Component {
    private Intake intake = null;
    protected Lift lift = null;
    protected boolean liftAuto = true;
    private Servo shoulderL = null;
    private Servo shoulderR = null;
    private Servo wrist = null;
    private Dropper dropper = null;
    private Servo launcher = null;
    private boolean handlerDeployed = false;
    private boolean handlerDeploying = false;
    private boolean handlerRetracting = false;

    private double shoulderRUpPos = 1.0;
    //    private double shoulderRMidPos = 0.5;
    private double shoulderRDownPos = 0.25;
    private double shoulderLUpPos = 0.95;
    //    private double shoulderLMidPos = 0.5;
    private double shoulderLDownPos = 0.1;

    // Value when full shoulder range is enabled
//    private double wristUpPos = 0.9;

    private double wristUpPos = 0.5;
    private double wristDownPos = 0.15;
    private double launcherLockPos = 0.75;
    private double launcherUnlockPos = 0.25;
    private boolean loading = false;
    ElapsedTime timer;
    protected boolean loggingOn;
    private LinearOpMode opMode;


    public Bot(HardwareMap hardwareMap, LinearOpMode opMode, boolean loggingOn) {
        super(opMode.telemetry, loggingOn);
        this.opMode = opMode;
        // Intake
        intake = new Intake(hardwareMap, telemetry, loggingOn);

        // Lift
        lift = new Lift(hardwareMap, telemetry, true);
        liftAuto = true;

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
        loading = false;
        handlerRetract();

        // Launcher
        launcher = hardwareMap.get(Servo.class, "launcher");
        launcher.setDirection(Servo.Direction.FORWARD);
        launcherLock();

        // Timer
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    public void dropPixel() {
        dropper.dropPixel();
    }

    public void handlerRetract() {
        lift.goToRetractPosition();
        handlerRetracting = true;
    }

    private void dropperRetract() {
        wrist.setPosition(wristDownPos);
        shoulderL.setPosition(shoulderLDownPos);
        shoulderR.setPosition(shoulderRDownPos);
        timer.reset();
    }

    private void liftMin() {
        lift.goToMinPosition();
        handlerDeployed = false;
        handlerRetracting = false;
    }

    public void handlerDeploy() {
        lift.goToDeployPosition();
        handlerDeploying = true;
    }

    private void dropperDeploy() {
        wrist.setPosition(wristUpPos);
        shoulderL.setPosition(shoulderLUpPos);
        shoulderR.setPosition(shoulderRUpPos);
        handlerDeploying = false;
        handlerDeployed = true;
    }


    public void load() {
        if (!handlerDeployed) {
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

    public void liftManualUp(double power) {
        if (handlerDeployed) {
            liftAuto = false;
            lift.manualUp(power);
        }
    }

    public void liftManualDown(double power) {
        if (handlerDeployed) {
            liftAuto = false;
            lift.manualDown(power);
        }
    }

    public void liftStop() {
        if (!liftAuto) {
            lift.stop();
            liftAuto = true;
        }
    }

    public void launcherLock() {
        launcher.setPosition(launcherLockPos);
    }

    public void launcherUnlock() {
        launcher.setPosition(launcherUnlockPos);
    }

    public void update() {
        if (!lift.isBusy()) {
            if (handlerDeploying) {
                dropperDeploy();
            } else if (handlerRetracting) {
                dropperRetract();
                if (timer.milliseconds() > 350)
                {
                    liftMin();
                }
            }
        }


        if (liftAuto) {
            lift.update();
        }

        intake.update();
        dropper.update();
        if (dropper.fullyLoaded() && loading) {
            loading = false;
            stopLoad();
        }
    }
}
