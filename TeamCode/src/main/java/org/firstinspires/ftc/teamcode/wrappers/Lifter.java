package org.firstinspires.ftc.teamcode.wrappers;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Lifter {
    public DcMotorEx lifterMotor;

    public static volatile int currentPosition = 0;

    public Servo dumpingBox;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public static volatile boolean lifter_running = false;

    public static double kP = 4;
    public static double kI = 0.0;
    public static double kD = 0.0;

    public enum LEVEL {
        DOWN(200), FIRST(15000), SECOND(25000), THIRD(37000); //third 35000
        public int ticks;

        LEVEL(int ticks) {
            this.ticks = ticks;
        }
    }

    public Lifter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        lifterMotor = hardwareMap.get(DcMotorEx.class, "lifter");
        dumpingBox = hardwareMap.get(Servo.class, "dumpingBox");

        lifterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lifterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        closeBox();
    }

    public void update() {
        currentPosition = lifterMotor.getCurrentPosition();
    }

    public void setLifterPower(double power) {
        lifterMotor.setPower(power);
    }

    public int getLifterPosition() {
        return currentPosition;
    }

    public void openBox() {
        dumpingBox.setPosition(0.8);
    }

    public void closeBox() {
        dumpingBox.setPosition(0.2);
    }

    public void openBox(long wait) {
        new Thread(() -> {
            if (Thread.currentThread().isInterrupted()) return; //paranoia

            if (wait != 0) {
                try {
                    Thread.sleep(wait);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            openBox();
        }).start();
    }

    public void closeBox(long wait) {
        new Thread(() -> {
            if (Thread.currentThread().isInterrupted()) return; //paranoia

            if (wait != 0) {
                try {
                    Thread.sleep(wait);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            closeBox();
        }).start();
    }


    public void intermediateBoxPosition() {
        dumpingBox.setPosition(0.5);
    }

    public void intermediateBoxPosition(long wait) {
        new Thread(() -> {
            if (Thread.currentThread().isInterrupted()) return; //paranoia

            if (wait != 0) {
                try {
                    Thread.sleep(wait);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            intermediateBoxPosition();
        }).start();
    }

    public void depositMineral(long wait) {
        new Thread(() -> {
            if (Thread.currentThread().isInterrupted()) return; //paranoia

            if (wait != 0) {
                try {
                    Thread.sleep(wait);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

            openBox();
            closeBox(500);

        }).start();
    }

    public void goToPosition(long waitFor, int targetPosition) {
        LifterThread lifterThread = new LifterThread(waitFor, targetPosition);
        Thread thread = new Thread(lifterThread);
        thread.start();
    }

    class LifterThread implements Runnable {
        long wait;
        int targetPosition;

        public LifterThread(long wait_ms, int targetPosition) {
            this.wait = wait_ms;
            this.targetPosition = targetPosition;
        }

        @Override
        public void run() {
            if (wait != 0) {
                try {
                    Thread.sleep(wait);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

            if (lifter_running) {
                telemetry.addLine("Already running");
                telemetry.update();
                return;
            }
            lifter_running = true;

            currentPosition = lifterMotor.getCurrentPosition();

            //We might already be at the target position
            if (Math.abs(currentPosition - targetPosition) < 1000) {
                lifter_running = false;
                return;
            }

            //go to target position with given coefficients
            PIDCoefficients coeffs = new PIDCoefficients(kP, kI, kD);
            PIDFController controller = new PIDFController(coeffs);
            controller.setTargetPosition(targetPosition);

            double initialError = Math.abs(targetPosition - currentPosition);
            double maxPower = 0.8;
            if (targetPosition < 20000) maxPower = 0.7;
            if (targetPosition < 5000) maxPower = 0.4;

            ElapsedTime timer = new ElapsedTime();
            timer.reset();
            while (!Thread.currentThread().isInterrupted()) {
                double correction = controller.update(currentPosition) / initialError;
                double sign = Math.signum(correction);
                double power = Range.clip(Math.abs(correction), 0.4, 1.0) * sign;
                lifterMotor.setPower(-Range.clip(power, -maxPower, maxPower));

//                telemetry.addData("current pos", currentPosition);
//                telemetry.addData("target pos", targetPosition);
//                telemetry.addData("correction", correction);
//                telemetry.addData("power", power);
//                telemetry.update();

                if (Math.abs(targetPosition - currentPosition) < 2000) break;

                if (timer.seconds() > 2.5) {
                    telemetry.log().clear();
                    telemetry.addLine("Thread aborted");
                    telemetry.update();
                    break;
                }
            }
            lifterMotor.setPower(0.0);
            telemetry.log().clear();
            lifter_running = false;
        }
    }

//    class LifterThread implements Runnable {
//        long wait;
//        int targetPosition;
//
//        public LifterThread(long wait_ms, int targetPosition) {
//            this.wait = wait_ms;
//            this.targetPosition = targetPosition;
//        }
//
//        @Override
//        public void run() {
//            if (wait != 0) {
//                try {
//                    Thread.sleep(wait);
//                } catch (InterruptedException e) {
//                    e.printStackTrace();
//                }
//            }
//
//            if (lifter_running) {
//                telemetry.addLine("Already running");
//                telemetry.update();
//                return;
//            }
//            lifter_running = true;
//
//            currentPosition = lifterMotor.getCurrentPosition();
//
//            //We might already be at the target position
//            if (Math.abs(currentPosition - targetPosition) < 10) {
//                lifter_running = false;
//                return;
//            }
//
//            //go to targetPosition
//            int direction = currentPosition > targetPosition ? -1 : 1;
//            lifterMotor.setTargetPosition(targetPosition);
//            lifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            lifterMotor.isBusy(); //this is so stupid
//
//            telemetry.log().clear();
//            telemetry.addData("Current Pos", currentPosition);
//            telemetry.addData("Target Pos", targetPosition);
//            telemetry.update();
//
//            try {
//                Thread.sleep(3000);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//
//            ElapsedTime timer = new ElapsedTime();
//            if (direction == -1) lifterMotor.setVelocity(-5000);
//            else lifterMotor.setVelocity(7000);
//            timer.reset();
//            while (lifterMotor.isBusy() && !Thread.currentThread().isInterrupted()) {
//                double currentDraw_AMPS = lifterMotor.getCurrent(CurrentUnit.AMPS);
//                telemetry.addData("Current draw (A)", currentDraw_AMPS);
//                if (currentDraw_AMPS > 15) {
//                    //current warning
//                    telemetry.addLine("Current Warning!");
//                }
//                telemetry.update();
//                if (timer.seconds() > 3.0) {
//                    telemetry.log().clear();
//                    telemetry.addLine("Something went wrong. Thread aborted");
//                    telemetry.update();
//                    break;
//                }
//            }
//            telemetry.log().clear();
//            lifterMotor.setVelocity(0.0);
//            lifter_running = false;
//        }
//    }
}