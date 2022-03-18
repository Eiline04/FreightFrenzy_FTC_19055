package org.firstinspires.ftc.teamcode.Auto.red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.RedWarehouseAuto;
import org.firstinspires.ftc.teamcode.Detection.CameraThread;
import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDriveImpl;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.wrappers.DuckMechanism;
import org.firstinspires.ftc.teamcode.wrappers.Intake;
import org.firstinspires.ftc.teamcode.wrappers.Lifter;
import org.firstinspires.ftc.teamcode.wrappers.TapeTurret;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Autonomous(name = "Red Carousel", group = "Red Auto")
public class RedCarouselAuto extends LinearOpMode {

    MecanumDriveImpl drive;
    Intake intake;
    Lifter lifter;
    TapeTurret turret;
    DuckMechanism duckMechanism;

    OpenCvCamera webcam;
    CameraThread cameraThread;
    Lifter.LEVEL result;

    static Pose2d startRedCarouselPose = new Pose2d(-40.085, -63.54, Math.toRadians(270.0));
    static Pose2d redCShippingHubPose = new Pose2d(-13.0, -45.5, Math.toRadians(260.0));
    static Pose2d carouselPose = new Pose2d(-54.3, -59.0, radians(270.0));

    @Override
    public void runOpMode() throws InterruptedException {

        lifter = new Lifter(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry, null);
        turret = new TapeTurret(hardwareMap);
        duckMechanism = new DuckMechanism(hardwareMap);

        Thread updater = new Thread(new RedCarouselAuto.Updater());

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


        TrajectorySequence carouselAuto =
                drive.trajectorySequenceBuilder(startRedCarouselPose)
                        //SPIN DUCK
                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                            duckMechanism.startSpin();
                        })
                        .lineToLinearHeading(carouselPose)
                        .waitSeconds(0.8)

                        //DELIVER PRELOAD
                        .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                            lifter.goToPosition(100, Lifter.LEVEL.THIRD.ticks);
                            lifter.intermediateBoxPosition(300);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(2.5, () -> {
                            lifter.depositMineral(0);
                            lifter.goToPosition(1000, Lifter.LEVEL.DOWN.ticks);
                        })

                        .UNSTABLE_addTemporalMarkerOffset(0.4, ()->{
                            intake.startIntakeVel(800);
                            intake.lowerIntake();

                        })

                        .setReversed(true)
                        .splineToSplineHeading(redCShippingHubPose, Math.toRadians(65.0))
                        .setReversed(false)

                        //COLLECT DUCK
                        .setVelConstraint(new TranslationalVelocityConstraint(20.0))
                        .splineToSplineHeading(new Pose2d(-24.8, -55.0, Math.toRadians(255.0)),Math.toRadians(180.0))//270
                        .setVelConstraint(new TranslationalVelocityConstraint(8.0))
                        .addTemporalMarker(()->{
                            duckMechanism.stopSpin();
                        })
                        .splineToSplineHeading(new Pose2d(-35.5, -56.5, Math.toRadians(240.0)), Math.toRadians(180.0))
                        .splineToSplineHeading(new Pose2d(-40.5, -55.5, Math.toRadians(250.0)), Math.toRadians(30.0))
                        .splineToSplineHeading(new Pose2d(-55.5, -53.4, Math.toRadians(240.0)), Math.toRadians(180.0))
                        .resetVelConstraint()


                        .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                            intake.raiseIntake();
                            intake.stopIntake();
                        })

                        .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                            lifter.goToPosition(100, Lifter.LEVEL.THIRD.ticks);
                             lifter.intermediateBoxPosition(300);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(1.8, () -> {
                            lifter.depositMineral(0);
                            lifter.goToPosition(1000, Lifter.LEVEL.DOWN.ticks);
                        })


                        //DELIVER DUCK
                        .setReversed(true)
                        .splineToSplineHeading(redCShippingHubPose, Math.toRadians(65.0))
                        .setReversed(false)

                        .splineToSplineHeading(new Pose2d(-61.0,-33.5,Math.toRadians(270.0)),Math.toRadians(120.0))
                        .waitSeconds(10)
                        .build();


        waitForStart();

        //detect go brr
        result = CameraThread.getResult();
        telemetry.addData("Result", result);
        telemetry.update();

        cameraThread.setState(CameraThread.CAMERA_STATE.KILL);

        updater.start(); //start calling update for intake and lifter

        //start action
        drive.setPoseEstimate(startRedCarouselPose);
        drive.followTrajectorySequence(carouselAuto);


    }

    //-----------TRAJECTORIES-----------


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

}
