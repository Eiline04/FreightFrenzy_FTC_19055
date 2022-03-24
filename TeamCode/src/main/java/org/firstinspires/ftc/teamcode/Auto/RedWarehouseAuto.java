package org.firstinspires.ftc.teamcode.Auto;

import android.content.Context;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

    static Pose2d startRedWareHousePose = new Pose2d(7.915, -63.54, Math.toRadians(270.0)); //x:11.6
    static Pose2d inRedWarehousePose = new Pose2d(47.0, -67.3, Math.toRadians(0.0));

    enum RedWarehouseShippingHub {
        FIRST_LEVEL(new Pose2d(-5.83, -44.5, Math.toRadians(280.0)), Lifter.LEVEL.FIRST),

        SECOND_LEVEL(new Pose2d(-5.83, -43.5, Math.toRadians(280.0)), Lifter.LEVEL.SECOND),

        THIRD_LEVEL(new Pose2d(-5.83, -43.0, Math.toRadians(285.0)), Lifter.LEVEL.THIRD); //y:-44.0

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

        TrajectorySequence preload = preload(startRedWareHousePose, RedWarehouseShippingHub.THIRD_LEVEL);
        TrajectorySequence cycles = cycles(preload.end(),0,0,0);
        TrajectorySequence secondCycle = cycles(cycles.end(),0,-0.5,0);
        TrajectorySequence thirdCycle = cycles(cycles.end(),1.0,-0.8,0);
        TrajectorySequence fourthCycle = cycles(cycles.end(),2.0,-1,0);
        TrajectorySequence park = park(cycles.end());

        waitForStart();

        //detect go brr
        result = CameraThread.getResult();
        telemetry.addData("Result", result);
        telemetry.update();

        cameraThread.setState(CameraThread.CAMERA_STATE.KILL);

        switch (result) {
            case FIRST:
                preload = preload(startRedWareHousePose, RedWarehouseShippingHub.FIRST_LEVEL);
                break;
            case SECOND:
                preload = preload(startRedWareHousePose, RedWarehouseShippingHub.SECOND_LEVEL);
                break;
            default:
                preload = preload(startRedWareHousePose, RedWarehouseShippingHub.THIRD_LEVEL);
                break;

        }

        updater.start(); //start calling update for intake and lifter

        drive.setPoseEstimate(startRedWareHousePose);
        drive.followTrajectorySequence(preload);

        //Cycle1
        drive.followTrajectorySequence(cycles(drive.getPoseEstimate(), -1.3,0,0));

        //Cycle2
        drive.followTrajectorySequence(secondCycle);

        //Cycle3
        drive.followTrajectorySequence(thirdCycle);

        //Cycle4
        drive.followTrajectorySequence(fourthCycle);

        //Park
        drive.followTrajectorySequence(park);

    }

    TrajectorySequence preload (Pose2d starPose, RedWarehouseShippingHub level){
        return drive.trajectorySequenceBuilder(starPose)
                //PRELOAD
                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                    lifter.goToPosition(100, level.level.ticks);
                    lifter.intermediateBoxPosition(300);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8,() -> {
                    lifter.depositMineral(0);
                    lifter.goToPosition(800, Lifter.LEVEL.DOWN.ticks);

                })
                .lineToLinearHeading(level.goTo)
                .build();
    }

    TrajectorySequence park(Pose2d currPose){
       return drive.trajectorySequenceBuilder(currPose)
               .lineToSplineHeading(new Pose2d(8.0, -61.5, radians(0))) //good one!
               .splineToLinearHeading(inRedWarehousePose, radians(0.0))

                .build();
    }

    TrajectorySequence cycles( Pose2d initialPose, double xAdd,double yAdd, double yCorrection ){
        return drive.trajectorySequenceBuilder(initialPose)

                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    intake.startIntake();
                    intake.lowerIntake();
                })


                .lineToSplineHeading(new Pose2d(7.9, -61.5, radians(0))) //good one!
                .splineToLinearHeading(new Pose2d(43 + xAdd, -67.0 + yAdd, radians(0.0)), radians(25.0))
                .waitSeconds(0.1)

                //deliver freight
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0.3,() -> {
                            intake.raiseIntake();
                            intake.stopIntake();
                        }
                )

                .splineToLinearHeading(new Pose2d(7.5, -67.2 , radians(0.0)), radians(  140.0))


                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lifter.goToPosition(100, RedWarehouseShippingHub.THIRD_LEVEL.level.ticks);
                    lifter.intermediateBoxPosition(300);
                })

                .UNSTABLE_addTemporalMarkerOffset(0.7,() -> {
                    lifter.depositMineral(0);
                    lifter.goToPosition(600, Lifter.LEVEL.DOWN.ticks);
                })
                .splineToSplineHeading(RedWarehouseShippingHub.THIRD_LEVEL.goTo, Math.toRadians(115.0))

                .setReversed(false)
                .build();
    }

    static double radians(double deg) {
        return Math.toRadians(deg);
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
