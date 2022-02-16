package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

    private double powCoeff = 0.6;
    private double triggerPow = 0.1;

    private boolean virgin = true;

    public static final boolean USING_ROADRUNNER = true;
    Pose2d startPose = new Pose2d(-40.085, -63.54, radians(270.0));
    Pose2d shippingHubPose = new Pose2d(-9.0 - 0.5 , -48.0 + 1.7, radians(265.0));

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
        baseServoPosition = 1.0;
        angleServoPosition = 0.0;

        while (opModeIsActive()) {
            controller1.update();
            controller2.update();

            intake.update();
            lifter.update();
            drive.update();

//            telemetry.addData("Ticks", lifter.getLifterPosition());
//            telemetry.addData("Vel", lifter.lifterMotor.getVelocity());


            //Base End Gae Pos
            if(controller1.XOnce()){
                baseServoPosition = 0.1;
                angleServoPosition = 0.1;
                turret.angleServo.setPosition(0.1);
                turret.baseServo.setPosition(0.1);
            }

            //Intake servos
            if (controller2.dpadDownOnce()) {
                intake.lowerIntake();
            }
            if (controller2.dpadUpOnce()) {
                intake.raiseIntake();
                intake.stopIntake(100);
            }

            //Intake Motor
            if (controller2.AOnce() && !gamepad2.start) {
                intake.lowerIntake();
                if (intake.direction == -1) {
                    intake.reverseIntake();
                }
                intake.startIntake(100);
            }
            if (controller2.BOnce() && !gamepad2.start) {
                intake.stopIntake();
            }
            if (controller2.XOnce()) {
                intake.stopIntake();
                sleep(100);
                intake.reverseIntake();
                intake.startIntake();
            }

            //Duck Mechanism
            if (controller2.YOnce()) {
//                start or stop duck motor
//                ----trebuie verificata pozitia lifetrului-----
//                if(virgin) {
//                    turret.baseServo.setPosition(0.1);
//                    baseServoPosition = 0.1;
//                    turret.angleServo.setPosition(0.08);
//                    angleServoPosition = 0.08;
//                }
                if (duckMechanism.running) {
                    duckMechanism.stopSpin();
                } else duckMechanism.startSpin();
            }

            //Tape Mechanism
            if (controller1.rightBumper()) {
                turret.startExtend();
            } else if (controller1.leftBumper()) {
                turret.startRetract();
            } else turret.stop();

            //Base servo limits: 0--right ; left--1

            if (controller1.dpadLeft()) {
                //move base left
                baseServoPosition = Range.clip(baseServoPosition + deltaBase, 0, 1.00);
                turret.setBasePos(baseServoPosition);
            }

            if (controller1.dpadRight()) {
                //move base right
                baseServoPosition = Range.clip(baseServoPosition - deltaBase, 0, 1.00);
                turret.setBasePos(baseServoPosition);
            }

            if (controller1.dpadUp()) {
                //move angle up
                angleServoPosition = Range.clip(angleServoPosition + deltaAngle, 0, 1.00);
                turret.setAnglePos(angleServoPosition);
            }

            if (controller1.dpadDown()) {
                //move angle down
                angleServoPosition = Range.clip(angleServoPosition - deltaAngle, 0, 1.00);
                turret.setAnglePos(angleServoPosition);
            }

            //-----LIFTER-----
            if (controller2.rightBumperOnce()) {
                turret.setBasePos(0.98);
                lifter.goToPosition(0, Lifter.LEVEL.THIRD.ticks);
                lifter.intermediateBoxPosition(200);
                lifter.depositMineral(500);
                lifter.goToPosition(1500, Lifter.LEVEL.DOWN.ticks);
            }

            if (controller2.leftBumperOnce()) {
                lifter.closeBox();
                lifter.goToPosition(0, Lifter.LEVEL.DOWN.ticks);
            }

            //----- LIL' FORWARD----

            if(controller1.left_trigger>0.05){
                triggerPow = controller1.left_trigger * powCoeff;
                drive.setMotorPowers(triggerPow,triggerPow,triggerPow,triggerPow);
            }

            if(controller1.right_trigger>0.05){
                triggerPow = controller1.right_trigger * -powCoeff;
                drive.setMotorPowers(triggerPow,triggerPow,triggerPow,triggerPow);
            }

            switch (currentMode) {
                case DRIVER_CONTROL:
                    double leftStickY = -controller1.left_stick_y;
                    double leftStickX = -controller1.left_stick_x;
                    double rotation = -controller1.right_stick_x;

                    drive.setDrivePower(new Pose2d(leftStickY, leftStickX, rotation));

                    if (controller1.YOnce() && USING_ROADRUNNER) {
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

            //telemetry.update();

            //-----------MANUAL CONTROL---------------
//            double lifterUp = controller2.right_trigger;
//            double lifterDown = controller2.left_trigger;
//
//            if (lifterUp > 0 && lifterDown == 0) {
//                //go up
//                lifter.setLifterPower(-lifterUp * 0.7);
//            } else {
//                if (lifterDown > 0 && lifterUp == 0) {
//                    //go down
//                    lifter.setLifterPower(lifterDown * 0.2);
//                } else {
//                    //stop
//                    lifter.setLifterPower(0.0);
//                }
//            }
        }

        PoseStorage.currentPose = new Pose2d(0,0); //reset pose storage
    }

    static double radians(double deg) {
        return Math.toRadians(deg);
    }
}
