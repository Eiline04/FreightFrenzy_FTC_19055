package org.firstinspires.ftc.teamcode.Auto.blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Detection.CameraThread;
import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDriveImpl;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.wrappers.DuckMechanism;
import org.firstinspires.ftc.teamcode.wrappers.Intake;
import org.firstinspires.ftc.teamcode.wrappers.Lifter;
import org.firstinspires.ftc.teamcode.wrappers.TapeTurret;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Autonomous(name = "Blue Warehouse", group = "Blue Auto")
public class BlueWarehouseAuto extends LinearOpMode {

    MecanumDriveImpl drive;
    Intake intake;
    Lifter lifter;
    TapeTurret turret;
    DuckMechanism duckMechanism;

    OpenCvCamera webcam;
    CameraThread cameraThread;
    Lifter.LEVEL result;

    ElapsedTime timer;

    static Pose2d startBlueWareHousePose = new Pose2d(7.915, 63.54, Math.toRadians(90.0));

    enum BlueWarehouseShippingHub {
        FIRST_LEVEL(new Pose2d(-5.9, 44.4, Math.toRadians(80.0)), Lifter.LEVEL.FIRST),

        SECOND_LEVEL(new Pose2d(-5.83, 44.5, Math.toRadians(80.0)), Lifter.LEVEL.SECOND),

        THIRD_LEVEL(new Pose2d(-5.83, 43.3, Math.toRadians(85.0)), Lifter.LEVEL.THIRD);

        Pose2d goTo;
        Lifter.LEVEL level;

        BlueWarehouseShippingHub(Pose2d goTo, Lifter.LEVEL level) {
            this.goTo = goTo;
            this.level = level;
        }
    }

    @Override
    public void runOpMode() {
        lifter = new Lifter(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry, null);
        turret = new TapeTurret(hardwareMap);
        duckMechanism = new DuckMechanism(hardwareMap);

        Thread updater = new Thread(new BlueWarehouseAuto.Updater());
        timer = new ElapsedTime();

        initWebcam();
        sleep(1000);
        cameraThread = new CameraThread(webcam);
        Thread cameraRunner = new Thread(cameraThread);
        cameraRunner.start();

        cameraThread.setState(CameraThread.CAMERA_STATE.INIT);
        sleep(1000);
        cameraThread.setState(CameraThread.CAMERA_STATE.STREAM);

        telemetry.addLine("Ready!");
        telemetry.update();

        drive = new MecanumDriveImpl(hardwareMap);

        TrajectorySequence preloadThird = preload(startBlueWareHousePose, BlueWarehouseShippingHub.THIRD_LEVEL);
        TrajectorySequence preloadSecond = preload(startBlueWareHousePose, BlueWarehouseShippingHub.SECOND_LEVEL);
        TrajectorySequence preloadFirst = preload(startBlueWareHousePose, BlueWarehouseShippingHub.FIRST_LEVEL);

        TrajectorySequence cycles = cycles(preloadThird.end(), 0, 0, 0);
        TrajectorySequence secondCycle = cycles(cycles.end(), -0.5, 0.3, 0);
        TrajectorySequence thirdCycle = cycles(cycles.end(), 1.2, 0.4, 0);
        TrajectorySequence fourthCycle = cycles(cycles.end(), 1.9, 0.5, 0);
        TrajectorySequence park = park(cycles.end());

        waitForStart();

        timer.reset();
        //detect go brr
        result = CameraThread.getBlueWarehouseResult();
        telemetry.addData("Result", result);
        telemetry.update();

        cameraThread.setState(CameraThread.CAMERA_STATE.KILL);

        updater.start(); //start calling update for intake and lifter
        drive.setPoseEstimate(startBlueWareHousePose);

        //Preload
        switch (result) {
            case FIRST:
                drive.followTrajectorySequence(preloadFirst);
                break;
            case SECOND:
                drive.followTrajectorySequence(preloadSecond);
                break;
            default:
                drive.followTrajectorySequence(preloadThird);
                break;

        }

        //Cycle1
        if (lifter.getLifterPosition() > 300) {
            lifter.closeBox(1000);
            lifter.goToPosition(1100, Lifter.LEVEL.DOWN.ticks);
        }
        drive.followTrajectorySequence(cycles(drive.getPoseEstimate(), -1.5, 0, 0));
//
//        //Cycle2
        drive.followTrajectorySequence(secondCycle);
//
//        //Cycle3
        drive.followTrajectorySequence(thirdCycle);
//
        //to verify if there is time for that
//        //Cycle4
        if(timer.seconds() < 23.7)
            drive.followTrajectorySequence(fourthCycle);
//
//        //Park
        drive.followTrajectorySequence(park);
        lifter.closeBox();
        lifter.goToPosition(50, Lifter.LEVEL.DOWN.ticks);

    }

    TrajectorySequence preload(Pose2d starPose, BlueWarehouseShippingHub level) {
        return drive.trajectorySequenceBuilder(starPose)
                //PRELOAD
                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                    lifter.goToPosition(100, level.level.ticks);
                    lifter.intermediateBoxPosition(500);
                })
                .addTemporalMarker(() -> {
                    lifter.depositMineral(1100);
                    lifter.goToPosition(1900, Lifter.LEVEL.DOWN.ticks);
                })
                .lineToLinearHeading(level.goTo)
                .build();
    }

    TrajectorySequence park(Pose2d currPose) {
        return drive.trajectorySequenceBuilder(currPose)
                .setVelConstraint(new TranslationalVelocityConstraint(60))
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    intake.startIntake();
                    intake.lowerIntake();
                })
                .lineToSplineHeading(new Pose2d(8.0, 61.5, radians(0))) //good one!
                .splineToLinearHeading(new Pose2d(47.0, 67.0, radians(0.0)), radians(330.0))
                .resetVelConstraint()
                .build();
    }

    TrajectorySequence cycles(Pose2d initialPose, double xAdd, double yAdd, double yCorrection) {
        return drive.trajectorySequenceBuilder(initialPose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lifter.closeBox(200);
                    lifter.goToPosition(300, Lifter.LEVEL.DOWN.ticks);
                })

                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    intake.startIntake();
                    intake.lowerIntake();
                })


                .lineToSplineHeading(new Pose2d(8.0, 61.5, radians(0))) //good one!
                .splineToLinearHeading(new Pose2d(43 + xAdd, 67.0 + yAdd, radians(0.0)), radians(330.0))
                .waitSeconds(0.08)

                //deliver freight
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                            intake.raiseIntake();
                            intake.stopIntake();
                        }
                )

                .splineToLinearHeading(new Pose2d(7, 68, radians(0.0)), radians(230.0))


                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lifter.goToPosition(300, BlueWarehouseShippingHub.THIRD_LEVEL.level.ticks);
                    lifter.intermediateBoxPosition(500);
                })

                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    lifter.depositMineral(0);
                    lifter.goToPosition(650, Lifter.LEVEL.DOWN.ticks);
                })
                .splineToSplineHeading(BlueWarehouseShippingHub.THIRD_LEVEL.goTo, Math.toRadians(270.0))//30
                .setReversed(false)
                .build();
    }

    static double radians(double deg) {
        return Math.toRadians(deg);
    }

    void resetLifter() {
        lifter.closeBox();
        lifter.goToPosition(100, Lifter.LEVEL.DOWN.ticks);
    }

    public void initWebcam() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
    }

    class Updater implements Runnable {
        @Override
        public void run() {
            while (opModeIsActive()) {
                lifter.update();
                intake.update();
                //telemetry.update();
            }
        }
    }

}
