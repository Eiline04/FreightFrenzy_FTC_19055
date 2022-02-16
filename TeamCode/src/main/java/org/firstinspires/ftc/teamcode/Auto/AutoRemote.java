package org.firstinspires.ftc.teamcode.Auto;

import android.content.Context;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
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

@Autonomous
public class AutoRemote extends LinearOpMode {
    MecanumDriveImpl drive;
    Intake intake;
    Lifter lifter;
    TapeTurret turret;
    DuckMechanism duckMechanism;

    OpenCvCamera webcam;
    CameraThread cameraThread;
    Lifter.LEVEL result;

    Pose2d startPose = new Pose2d(-40.085, -63.54, radians(270.0));
    Pose2d shippingHubPose = new Pose2d(-9.0 + 2.0, -48.0 + 1.12, radians(260.0));

    @Override
    public void runOpMode() throws InterruptedException {

        deleteCache(AppUtil.getDefContext());

        lifter = new Lifter(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry, null);
        turret = new TapeTurret(hardwareMap);
        duckMechanism = new DuckMechanism(hardwareMap);

        Thread updater = new Thread(new Updater());

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

        waitForStart();

        //detect go brr
        result = CameraThread.getResult();
        telemetry.addData("Result", result);
        telemetry.update();

        cameraThread.setState(CameraThread.CAMERA_STATE.KILL);

        updater.start(); //start calling update for intake and lifter

        TrajectorySequence duck = duck();
        TrajectorySequence three = levelThreePreload(duck.end());
        TrajectorySequence two = levelTwoPreload(duck.end());
        TrajectorySequence one = levelOnePreload(duck.end());

        drive.setPoseEstimate(startPose);

        drive.followTrajectorySequence(duck);

        if (result == Lifter.LEVEL.FIRST) {
            drive.followTrajectorySequence(one);

            lifter.closeBox();
            lifter.goToPosition(700, Lifter.LEVEL.DOWN.ticks);
            duckMechanism.stopSpin();
            intake.raiseIntake(800);

        } else if (result == Lifter.LEVEL.SECOND) {
            drive.followTrajectorySequence(two);

            lifter.closeBox();
            lifter.goToPosition(300, Lifter.LEVEL.DOWN.ticks);
            duckMechanism.stopSpin();

        } else {
            drive.followTrajectorySequence(three);

            lifter.closeBox();
            lifter.goToPosition(0, Lifter.LEVEL.DOWN.ticks);
            duckMechanism.stopSpin();
        }

        drive.followTrajectorySequence(cycles(drive.getPoseEstimate()));

        PoseStorage.currentPose = drive.getPoseEstimate();
    }

    TrajectorySequence cycles(Pose2d initialPose) {
        return drive.trajectorySequenceBuilder(initialPose)
                //START OF CYCLE 1
                .lineToLinearHeading(new Pose2d(2.0, -73.0, radians(-5.0)))
                .addDisplacementMarker(() -> {
                    intake.lowerIntake();
                    intake.startIntake();
                })

                //.setVelConstraint(new TranslationalVelocityConstraint(35.0))
                .lineToLinearHeading(new Pose2d(50.0 - 6.0, -74.0, radians(0.0)))
                .waitSeconds(0.1)

                //go back now
                .resetVelConstraint()
                .setReversed(true)

                //.setVelConstraint(new TranslationalVelocityConstraint(35.0))
                .lineToLinearHeading(new Pose2d(2.0, -74.0, radians(0.0))).addDisplacementMarker(() -> {
                    intake.raiseIntake();
                    intake.stopIntake();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> lifter.goToPosition(0, Lifter.LEVEL.THIRD.ticks))

                .setVelConstraint(new TranslationalVelocityConstraint(50.0))
                .lineToLinearHeading(shippingHubPose)
                .resetVelConstraint()

                .UNSTABLE_addTemporalMarkerOffset(0, () -> lifter.dumpingBox.setPosition(0.8))
                .waitSeconds(0.4)
                .addDisplacementMarker(() -> {
                    lifter.closeBox();
                    lifter.goToPosition(0, Lifter.LEVEL.DOWN.ticks);
                })
                .setReversed(false)
                .resetVelConstraint()


                //START OF CYCLE 2
                .lineToLinearHeading(new Pose2d(2.0, -73.0, radians(-5.0)))
                .addDisplacementMarker(() -> {
                    intake.lowerIntake();
                    intake.startIntake();
                })
                .setVelConstraint(new TranslationalVelocityConstraint(50.0))
                .lineToLinearHeading(new Pose2d(50.0 - 4.0, -74.0, radians(0.0)))
                .waitSeconds(0.1)

                //go back now
                .setReversed(true)

                .setVelConstraint(new TranslationalVelocityConstraint(50.0))
                .lineToLinearHeading(new Pose2d(2.0, -74.0, radians(0.0))).addDisplacementMarker(() -> {
                    intake.raiseIntake();
                    intake.stopIntake();
                })
                .resetVelConstraint()
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> lifter.goToPosition(0, Lifter.LEVEL.THIRD.ticks))

                //.lineToLinearHeading(shippingHubPose)
                .setVelConstraint(new TranslationalVelocityConstraint(50.0))

                .lineToLinearHeading(new Pose2d(-9.0 + 0.5, -48.0 + 1.12, radians(260.0)))
                .resetVelConstraint()
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lifter.dumpingBox.setPosition(0.8))
                .waitSeconds(0.4)
                .addDisplacementMarker(() -> {
                    lifter.closeBox();
                    lifter.goToPosition(0, Lifter.LEVEL.DOWN.ticks);
                })
                .setReversed(false)
                .resetVelConstraint()

                //PARK
                .lineToLinearHeading(new Pose2d(2.0, -74.0, radians(-5.0)))
                .addDisplacementMarker(() -> {
                    intake.raiseIntake();
                    intake.stopIntake();
                })
                .setVelConstraint(new TranslationalVelocityConstraint(50.0))
                .lineToLinearHeading(new Pose2d(50.0 - 7.0, -75.0, radians(0.0)))
                .build();
    }

    TrajectorySequence duck() {
        return drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> duckMechanism.startSpin())
                .setVelConstraint(new TranslationalVelocityConstraint(40.0))
                .lineToLinearHeading(new Pose2d(-55.23, -59.0, radians(270.0)))
                .waitSeconds(0.7)
                .build();
    }

    TrajectorySequence levelThreePreload(Pose2d initialPose) {
        return drive.trajectorySequenceBuilder(initialPose)
                .setVelConstraint(new TranslationalVelocityConstraint(30.0))
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> lifter.goToPosition(0, Lifter.LEVEL.THIRD.ticks))
                .lineToLinearHeading(new Pose2d(-9.0, -48.0, radians(265.0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lifter.dumpingBox.setPosition(0.8))
                .waitSeconds(0.4)
                .resetVelConstraint()
                .build();
    }

    TrajectorySequence levelTwoPreload(Pose2d initialPose) {
        return drive.trajectorySequenceBuilder(initialPose)
                .setVelConstraint(new TranslationalVelocityConstraint(30.0))
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> lifter.goToPosition(0, Lifter.LEVEL.SECOND.ticks))
                .lineToLinearHeading(new Pose2d(-9.0, -48.0, radians(265.0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lifter.dumpingBox.setPosition(0.8))
                .waitSeconds(0.4)
                .resetVelConstraint()
                .build();
    }

    TrajectorySequence levelOnePreload(Pose2d initialPose) {
        return drive.trajectorySequenceBuilder(initialPose)
                .setVelConstraint(new TranslationalVelocityConstraint(30))
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    lifter.goToPosition(0, Lifter.LEVEL.FIRST.ticks);
                    intake.lowerIntake();
                })
                .lineToLinearHeading(new Pose2d(-9.0 - 2.0, -48.0, radians(265.0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lifter.dumpingBox.setPosition(0.8))
                .waitSeconds(0.4)
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
        } catch (Exception e) { e.printStackTrace();}
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
        } else if(dir!= null && dir.isFile()) {
            return dir.delete();
        } else {
            return false;
        }
    }

}

//        telemetry.addLine("Expected pose");
//        telemetry.addData("x", expectedPose.getX());
//        telemetry.addData("y", expectedPose.getY());
//        telemetry.addData("angle", Math.toDegrees(expectedPose.getHeading()));
//
//        telemetry.addLine();
//        telemetry.addLine("Actual pose");
//        telemetry.addData("x", finalPose.getX());
//        telemetry.addData("y", finalPose.getY());
//        telemetry.addData("angle", Math.toDegrees(finalPose.getHeading()));
//        telemetry.update();
//
//        sleep(20000);