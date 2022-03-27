package org.firstinspires.ftc.teamcode.Auto.red;

import android.content.Context;

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

import java.io.File;
import java.sql.Time;

@Autonomous(name = "Red Warehouse", group = "Red Auto")
public class RedWarehouseAuto extends LinearOpMode {

    MecanumDriveImpl drive;
    Intake intake;
    Lifter lifter;
    TapeTurret turret;
    DuckMechanism duckMechanism;

    OpenCvCamera webcam;
    CameraThread cameraThread;
    Lifter.LEVEL result;

    ElapsedTime timer;

    static Pose2d startRedWareHousePose = new Pose2d(7.915, -63.54, Math.toRadians(270.0)); //x:11.6
    static Pose2d inRedWarehousePose = new Pose2d(47.0, -67.3, Math.toRadians(0.0));

    enum RedWarehouseShippingHub {
        FIRST_LEVEL(new Pose2d(-5.83, -44.5, Math.toRadians(280.0)), Lifter.LEVEL.FIRST),

        SECOND_LEVEL(new Pose2d(-5.83, -43.5, Math.toRadians(280.0)), Lifter.LEVEL.SECOND),

        THIRD_LEVEL(new Pose2d(-5.83, -43.2, Math.toRadians(285.0)), Lifter.LEVEL.THIRD); //y:-44.0

        Pose2d goTo;
        Lifter.LEVEL level;

        RedWarehouseShippingHub(Pose2d goTo, Lifter.LEVEL level) {
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

        Thread updater = new Thread(new RedWarehouseAuto.Updater());
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

        TrajectorySequence preloadThird = preload(startRedWareHousePose, RedWarehouseShippingHub.THIRD_LEVEL);
        TrajectorySequence preloadSecond = preload(startRedWareHousePose, RedWarehouseShippingHub.SECOND_LEVEL);
        TrajectorySequence preloadFirst = preload(startRedWareHousePose, RedWarehouseShippingHub.FIRST_LEVEL);

        TrajectorySequence cycles = cycles(preloadThird.end(), 0, 0, 0);
        TrajectorySequence secondCycle = cycles(cycles.end(), -0.5, -0.5, 0);
        TrajectorySequence thirdCycle = cycles(cycles.end(), 1.0, -0.8, 0);
        TrajectorySequence fourthCycle = cycles(cycles.end(), 1.9, -1, 0);
        TrajectorySequence park = park(cycles.end());

        waitForStart();

        timer.reset();
        //detect go brr
        result = CameraThread.getResult();
        telemetry.addData("Result", result);
        telemetry.update();

        cameraThread.setState(CameraThread.CAMERA_STATE.KILL);

        updater.start(); //start calling update for intake and lifter
        drive.setPoseEstimate(startRedWareHousePose);

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
            lifter.closeBox(300);
            lifter.goToPosition(500, Lifter.LEVEL.DOWN.ticks);
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
        if(timer.seconds() < 24)
        {drive.followTrajectorySequence(fourthCycle);}
//
//        //Park
        drive.followTrajectorySequence(park);
        lifter.closeBox();
        lifter.goToPosition(50, Lifter.LEVEL.DOWN.ticks);

    }

    TrajectorySequence preload(Pose2d starPose, RedWarehouseShippingHub level) {
        return drive.trajectorySequenceBuilder(starPose)
                //PRELOAD
                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                    lifter.goToPosition(100, level.level.ticks);
                    lifter.intermediateBoxPosition(500);
                })
                .addTemporalMarker(() -> {
                    lifter.depositMineral(1100);
                    lifter.goToPosition(1800, Lifter.LEVEL.DOWN.ticks);
                })
                .lineToLinearHeading(level.goTo)
                .build();
    }

    TrajectorySequence park(Pose2d currPose) {
        return drive.trajectorySequenceBuilder(currPose)
                .setVelConstraint(new TranslationalVelocityConstraint(60))
                .lineToSplineHeading(new Pose2d(8.0, -61.5, radians(0))) //good one!
                .splineToLinearHeading(new Pose2d(44, -67.0, radians(0.0)), radians(25.0))
                .resetVelConstraint()
                .build();
    }

    TrajectorySequence cycles(Pose2d initialPose, double xAdd, double yAdd, double yCorrection) {
        return drive.trajectorySequenceBuilder(initialPose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lifter.closeBox();
                    lifter.goToPosition(50, Lifter.LEVEL.DOWN.ticks);
                })

                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    intake.startIntake();
                    intake.lowerIntake();
                })


                .lineToSplineHeading(new Pose2d(7.9, -61.5, radians(0))) //good one!
                .splineToLinearHeading(new Pose2d(43 + xAdd, -67.0 + yAdd, radians(0.0)), radians(25.0))
                .waitSeconds(0.08)//0.1

                //deliver freight
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                            intake.raiseIntake();
                            intake.stopIntake();
                        }
                )

                .splineToLinearHeading(new Pose2d(7.5, -67.2, radians(0.0)), radians(140.0))


                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lifter.goToPosition(300, RedWarehouseShippingHub.THIRD_LEVEL.level.ticks);
                    lifter.intermediateBoxPosition(500);
                })

                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    lifter.depositMineral(0);
                    lifter.goToPosition(650, Lifter.LEVEL.DOWN.ticks);
                })
                .splineToSplineHeading(RedWarehouseShippingHub.THIRD_LEVEL.goTo, Math.toRadians(115.0))
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

    public static void deleteCache(Context context) {
        try {
            File dir = context.getCacheDir();
            deleteDir(dir);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public static boolean deleteDir(File dir) {
        if (dir != null && dir.isDirectory()) {
            String[] children = dir.list();
            for (int i = 0; i < children.length; i++) {
                boolean success = deleteDir(new File(dir, children[i]));
                if (!success) {
                    return false;
                }
            }
            return dir.delete();
        } else if (dir != null && dir.isFile()) {
            return dir.delete();
        } else {
            return false;
        }
    }
}