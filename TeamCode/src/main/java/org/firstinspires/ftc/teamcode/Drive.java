package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Auto.AutoRemote;
import org.firstinspires.ftc.teamcode.Auto.PoseStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDriveImpl;
import org.firstinspires.ftc.teamcode.utilities.ControllerInput;
import org.firstinspires.ftc.teamcode.wrappers.DuckMechanism;
import org.firstinspires.ftc.teamcode.wrappers.Intake;
import org.firstinspires.ftc.teamcode.wrappers.Lifter;
import org.firstinspires.ftc.teamcode.wrappers.TapeTurret;

@TeleOp
public class Drive extends LinearOpMode {

    ControllerInput controller1, controller2;

    MecanumDriveImpl drive;
    Intake intake;
    DuckMechanism duckMechanism;
    TapeTurret turret;
    Lifter lifter;

    public double baseServoPosition, angleServoPosition;
    public double deltaBase = 0.008, deltaAngle = 0.01;

    private double powCoeff = 1.5;
    private double triggerPow = 0.1;

    private boolean virgin = true;
    public static int forward = 1;

    public static final boolean USING_ROADRUNNER = true;
    Pose2d startPose = new Pose2d(-40.085, -63.54, radians(270.0));
    Pose2d shippingHubPose = new Pose2d(-9.0 - 0.5, -48.0 + 1.7, radians(265.0));

    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;

    @Override
    public void runOpMode() {


        AutoRemote.deleteCache(AppUtil.getDefContext());

        drive = new MecanumDriveImpl(hardwareMap);
        intake = new Intake(hardwareMap, telemetry, gamepad2);
        duckMechanism = new DuckMechanism(hardwareMap);
        turret = new TapeTurret(hardwareMap);
        lifter = new Lifter(hardwareMap, telemetry);

        controller1 = new ControllerInput(gamepad1);
        controller2 = new ControllerInput(gamepad2);

        telemetry.addLine("Ready!");
        telemetry.update();

        virgin = true;
        forward = 1;

        if (USING_ROADRUNNER) {
            Pose2d storedPose = PoseStorage.currentPose;
            drive.setPoseEstimate(storedPose);
            if (PoseStorage.currentPose.getX() == 0 && PoseStorage.currentPose.getY() == 0) {
                telemetry.addLine("Nicio pozitie stocata");
                telemetry.update();
                drive.setPoseEstimate(startPose);
            }
        }

        waitForStart();
        intake.raiseIntake();
        baseServoPosition = 0.85;
        angleServoPosition = 0.0;

        while (opModeIsActive()) {
            controller1.update();
            controller2.update();

            intake.update();
            lifter.update();
            drive.update();

            ///---------1. FIRST CONTROLLER--------

            //--BASE END GAME auto--
            if (controller1.AOnce() && !gamepad1.start) {
                if (lifter.getLifterPosition() < 250) {
                    baseServoPosition = 0.1;
                    angleServoPosition = 0.1;
                    turret.angleServo.setPosition(0.1);
                    turret.baseServo.setPosition(0.1);
                }
            }

            //--WHEEL DIRECTION--
            if (controller1.rightBumperOnce()) {
                changeDirection();
            }
            if (controller1.leftBumperOnce()) {
                intakeIsFront();
            }

            //--DUCK MECHANISM--
            if (controller1.YOnce()) {
                if (duckMechanism.running) {
                    duckMechanism.stopSpin();
                } else duckMechanism.startPowSpin(0.5);
            }

            //--LIL' FORWARD--
            if (controller1.left_trigger > 0.02) {
                triggerPow = controller1.left_trigger * powCoeff * forward;
                drive.setMotorPowers(triggerPow, triggerPow, triggerPow, triggerPow);
            }

            if (controller1.right_trigger > 0.02) {
                triggerPow = controller1.right_trigger * powCoeff * -forward;
                drive.setMotorPowers(triggerPow, triggerPow, triggerPow, triggerPow);
            }

            //-----------------------------------


            ///---------2. SECOND CONTROLLER--------

            //--INTAKE--
            //Intake servos
            if (controller2.YOnce()) {
                if (Intake.intakeUp) {
                    intake.lowerIntake();
                } else {
                    intake.raiseIntake();
                    intake.stopIntake(100);
                }
            }

            //Intake Motor
            if (controller2.AOnce() && !gamepad2.start) {
                if (!Intake.intakeIsWorking) {
                    intake.lowerIntake();
                    if (intake.direction == -1) {
                        intake.reverseIntake();
                    }
                    intake.startIntake(100);
                } else intake.stopIntake();
            }

            //for shared shipping hub
            if (controller2.BOnce() && !gamepad2.start) {
                turret.setAnglePos(0);
                turret.setBasePos(0.98);
                lifter.goToPosition(100, 21500);
                intake.setIntakePosition(0.4);
                lifter.waitGoToBoxPosition(600, 0.85);
                //lifter.depositMineral(600);
                lifter.closeBox(1200);
                lifter.goToPosition(1500, Lifter.LEVEL.DOWN.ticks);
                intake.raiseIntake(1800);
            }
            if (controller2.XOnce()) {
                intake.stopIntake();
                sleep(100);
                intake.reverseIntake();
                intake.startIntake();
            }

            //--TAPE MECHANISM--
            double tapeExtendCoeff = 0.9;
            double tapeRetractCoeff = 0.6;
            if (controller2.right_trigger > 0.1) {
                //turret.startExtend();
                turret.extender.setPower(Range.clip(controller2.right_trigger, 0, 1));
            } else if (controller2.left_trigger > 0.1) {
                turret.extender.setPower(Range.clip(-controller2.left_trigger, -1, 0) * tapeRetractCoeff);
                //turret.startRetract();
            } else turret.stop();

            //Base servo limits: 0--right ; left--1

            if (controller2.dpadLeft()) {
                //move base left
                baseServoPosition = Range.clip(baseServoPosition + deltaBase, 0, 1.00);
                turret.setBasePos(baseServoPosition);
            }

            if (controller2.dpadRight()) {
                //move base right
                baseServoPosition = Range.clip(baseServoPosition - deltaBase, 0, 1.00);
                turret.setBasePos(baseServoPosition);
            }

            if (controller2.dpadUp()) {
                //move angle up
                angleServoPosition = Range.clip(angleServoPosition + deltaAngle, 0, 1.00);
                turret.setAnglePos(angleServoPosition);
            }

            if (controller2.dpadDown()) {
                //move angle down
                angleServoPosition = Range.clip(angleServoPosition - deltaAngle, 0, 1.00);
                turret.setAnglePos(angleServoPosition);
            }

            //--LIFTER--
            if (controller2.rightBumperOnce() && !controller2.leftBumperOnce()) {
                turret.setBasePos(0.98);
                lifter.goToPosition(100, Lifter.LEVEL.THIRD.ticks);
                lifter.intermediateBoxPosition(300);
                lifter.depositMineral(600);
                lifter.goToPosition(1600, Lifter.LEVEL.DOWN.ticks);
            }

            if (controller2.leftBumperOnce() && !controller2.rightBumperOnce()) {
                turret.setBasePos(0.98);
                lifter.closeBox();
                lifter.goToPosition(0, Lifter.LEVEL.DOWN.ticks);
            }

            //-----------------------------------

            switch (currentMode) {
                case DRIVER_CONTROL:
                    handleDriving();

                    if (controller1.XOnce() && USING_ROADRUNNER) {
                        //generate a spline and follow it
                        Trajectory trajectory = drive.trajectoryBuilder(drive.getPoseEstimate()).lineToLinearHeading(shippingHubPose).build();
                        intake.raiseIntake();
                        intake.stopIntake();

                        drive.followTrajectoryAsync(trajectory);
                        currentMode = Mode.AUTOMATIC_CONTROL;
                    }

                    break;
                case AUTOMATIC_CONTROL:
                    if (controller1.BOnce() && USING_ROADRUNNER) {
                        //cancel following
                        drive.breakFollowing();
                    }

                    if (!drive.isBusy() && USING_ROADRUNNER) {
                        //give control back to drivers
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    break;
            }

        }

        PoseStorage.currentPose = new Pose2d(0, 0); //reset pose storage

    }

    private void handleDriving() {
        boolean leftArrow = controller1.dpadLeft();
        boolean rightArrow = controller1.dpadRight();
        boolean forwardArrow = controller1.dpadUp();
        boolean backwardArrow = controller1.dpadDown();

        if (leftArrow || rightArrow || forwardArrow || backwardArrow) {
            double power = 0.7;
            double forward = 0, strafe = 0;
            if (leftArrow) strafe = -power;
            if (rightArrow) strafe = power;
            if (forwardArrow) forward = power;
            if (backwardArrow) forward = -power;
            drive.setDrivePower(new Pose2d(forward, strafe, 0));

        } else {
            double leftStickY = -controller1.left_stick_y;
            double leftStickX = -controller1.left_stick_x;
            double rotation = -controller1.right_stick_x;
            drive.setDrivePower(new Pose2d(leftStickY, leftStickX, rotation));
        }
    }

    private void changeDirection() {
        forward *= -1;
    }

    private void intakeIsFront() {
        forward = 1;
    }

    static double radians(double deg) {
        return Math.toRadians(deg);
    }

}
