package org.firstinspires.ftc.teamcode.wrappers;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake {
    public DcMotorEx intake;
    public int direction = 1;
    public static volatile boolean running = false;

    public Servo leftServo;
    public Servo rightServo;
    public Servo intakeServo;

    //public DistanceSensor distanceSensor;
    public Rev2mDistanceSensor distanceSensor;
    private volatile double rawDistance;
    public volatile boolean enableWatchdog = true;

    public static double DISTANCE_THRESHOLD = 4; //4 // best6
    public static long WATCHDOG_DELAY = 2000; //ms
    public ElapsedTime timer;

    public double velocity = 1000;
    public static boolean intakeUp = true;
    public static boolean intakeIsWorking = false;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private Gamepad gamepad;



    public Intake(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        if (gamepad != null) this.gamepad = gamepad;

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        leftServo = hardwareMap.get(Servo.class, "intakeLeftServo");
        rightServo = hardwareMap.get(Servo.class, "intakeRightServo");
        intakeServo = hardwareMap.get(Servo.class, "cupaServo");

        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setPower(0.0);
        intake.setDirection(DcMotorEx.Direction.FORWARD);

        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor");
        timer = new ElapsedTime();

        intakeUp = true;
        intakeIsWorking = false;

        initIntake();
        timer.reset();
    }

    public void update() {
        if (!enableWatchdog) return;
        rawDistance = distanceSensor.getDistance(DistanceUnit.CM);


        if (timer.milliseconds() < WATCHDOG_DELAY) return;

        if (rawDistance < DISTANCE_THRESHOLD && rawDistance != 0.0) {
            raiseIntake();
            intake.setVelocity(velocity * 0.5);
            stopIntake(400);
            timer.reset();
            if (gamepad != null) gamepad.rumble(200);
        }
    }

    public void setIntakePosition(double pos) {
        leftServo.setPosition(pos);
        rightServo.setPosition(1 - pos);
    }

    public void initIntake() {
        releaseElements();
        setIntakePosition(0.52);
    }

    public void raiseIntake() {
        intakeUp = true;
        setIntakePosition(0.48);
        releaseElements();
    }

    public void lowerIntake() {
        intakeUp = false;
        setIntakePosition(0.169);
        blockElements();
    }

    public void startIntake() {
        intake.setVelocity(velocity * direction);
        intakeIsWorking = true;
    }

    public void stopIntake() {
        intake.setVelocity(0.0);
        intakeIsWorking = false;
    }

    public void startIntake(long wait) {
        new Thread(() -> {
            if (Thread.currentThread().isInterrupted()) return; //paranoia

            if (wait != 0) {
                try {
                    Thread.sleep(wait);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            startIntake();
        }).start();
    }

    public void stopIntake(long wait) {
        new Thread(() -> {
            if (Thread.currentThread().isInterrupted()) return; //paranoia

            if (wait != 0) {
                try {
                    Thread.sleep(wait);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            telemetry.addData("Distance", rawDistance);
            telemetry.update();
            stopIntake();
        }).start();
    }

    public void setIntakePosition(long wait, double pos) {
        new Thread(() -> {
            if (running) return; //if another thread is running don't use this one
            running = true;
            if (Thread.currentThread().isInterrupted()) return; //paranoia

            if (wait != 0) {
                try {
                    Thread.sleep(wait);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            setIntakePosition(pos);
            running = false;
        }).start();
    }

    public void raiseIntake(long wait) {
        new Thread(() -> {
            if (running) return; //if another thread is running don't use this one
            running = true;
            if (Thread.currentThread().isInterrupted()) return; //paranoia

            if (wait != 0) {
                try {
                    Thread.sleep(wait);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            raiseIntake();
            running = false;
        }).start();
    }

    public void lowerIntake(long wait) {
        new Thread(() -> {
            if (running) return; //if another thread is running don't use this one
            running = true;
            if (Thread.currentThread().isInterrupted()) return; //paranoia

            if (wait != 0) {
                try {
                    Thread.sleep(wait);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            lowerIntake();
            running = false;
        }).start();
    }

    public void releaseElements() {
        intakeServo.setPosition(1.0);
    }

    public void blockElements() {
        intakeServo.setPosition(0.3);
    }

    public void blockElements(long wait) {
        new Thread(() -> {
            if (Thread.currentThread().isInterrupted()) return; //paranoia

            if (wait != 0) {
                try {
                    Thread.sleep(wait);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            blockElements();
        }).start();
    }

    public void releaseElements(long wait) {
        new Thread(() -> {
            if (Thread.currentThread().isInterrupted()) return; //paranoia

            if (wait != 0) {
                try {
                    Thread.sleep(wait);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            releaseElements();
        }).start();
    }

    public void reverseIntake() {
        direction *= -1;
    }
}