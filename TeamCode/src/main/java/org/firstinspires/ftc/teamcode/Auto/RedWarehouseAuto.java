package org.firstinspires.ftc.teamcode.Auto;

import android.content.Context;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
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

@Autonomous(name = "RedWarehouse")
public class RedWarehouseAuto extends LinearOpMode {

    MecanumDriveImpl drive;
    Intake intake;
    Lifter lifter;
    TapeTurret turret;
    DuckMechanism duckMechanism;

    OpenCvCamera webcam;
    CameraThread cameraThread;
    Lifter.LEVEL result;

    static Pose2d startWareHousePose = new Pose2d(7.915, -63.54, Math.toRadians(270.0)); //x:11.6
    static Pose2d startCarouselPose = new Pose2d(-40.085, -63.54, Math.toRadians(270.0));
    static Pose2d shippingHubPose = new Pose2d(-5.83, -44.5, Math.toRadians(280.0));//x:-9.0
    static Pose2d inWarehousePose = new Pose2d(47.0, -66.3, Math.toRadians(0.0));


    @Override
    public void runOpMode() {
        //deleteCache(AppUtil.getDefContext());

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

        TrajectorySequence warehouseShippingHub =
                drive.trajectorySequenceBuilder(startWareHousePose)

                        .setVelConstraint(new TranslationalVelocityConstraint(50.00))
                        //PRELOAD
                        .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                            lifter.goToPosition(100, Lifter.LEVEL.THIRD.ticks);
                            lifter.intermediateBoxPosition(300);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(0.75,() -> {
                            lifter.depositMineral(0);
                            lifter.goToPosition(1000, Lifter.LEVEL.DOWN.ticks);
                        })
                        .lineToLinearHeading(shippingHubPose)
                        .resetVelConstraint()
                        .build();


        waitForStart();

        //detect go brr
        result = CameraThread.getResult();
        telemetry.addData("Result", result);
        telemetry.update();

        cameraThread.setState(CameraThread.CAMERA_STATE.KILL);


        updater.start(); //start calling update for intake and lifter

        drive.setPoseEstimate(startWareHousePose);
        drive.followTrajectorySequence(warehouseShippingHub);

        //Cycle1
        drive.followTrajectorySequence(cycles(drive.getPoseEstimate(), 0,0));

        //Cycle2
        drive.followTrajectorySequence(cycles(drive.getPoseEstimate(), 0.3,-0.1));

        //Cycle3
        drive.followTrajectorySequence(cycles(drive.getPoseEstimate(), 0.45,-0.15));

        //Park
        drive.followTrajectorySequence(park(drive.getPoseEstimate()));

    }

    TrajectorySequence park(Pose2d currPose){
       return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .splineToSplineHeading(new Pose2d(23.3, -66.3, radians(0.0)), radians(0.0))//347
                .splineToSplineHeading(inWarehousePose, radians(0.0))

                .build();
    }

    TrajectorySequence cycles( Pose2d initialPose, double xAdd,double yAdd ){
        return drive.trajectorySequenceBuilder(initialPose)

                .addTemporalMarker(0.9, () -> {
                    intake.startIntake();
                    intake.lowerIntake();
                })

                .splineToSplineHeading(new Pose2d(23.3, -66.3 + yAdd, radians(0.0)), radians(0.0))//347

                //.setVelConstraint(new TranslationalVelocityConstraint(50.00))

                //go to collect freight
                .splineToSplineHeading(new Pose2d(47.0 + xAdd, -66.3), radians(0.0))
                .waitSeconds(0.2)

                //deliver freight
                .resetVelConstraint()
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0.3,() -> {
                            intake.raiseIntake();
                            intake.stopIntake();
                        }
                )
                .splineToSplineHeading(new Pose2d(32.0, -66.5, radians(0.0)), radians(180.0))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lifter.goToPosition(100, Lifter.LEVEL.THIRD.ticks);
                    lifter.intermediateBoxPosition(300);
                })

                .splineToSplineHeading(shippingHubPose, radians(90.0))

                .addTemporalMarker(() -> {
                    lifter.depositMineral(0);
                    lifter.goToPosition(1000, Lifter.LEVEL.DOWN.ticks);
                })
                .waitSeconds(0.5)
                .setReversed(false)
                .resetVelConstraint()
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
