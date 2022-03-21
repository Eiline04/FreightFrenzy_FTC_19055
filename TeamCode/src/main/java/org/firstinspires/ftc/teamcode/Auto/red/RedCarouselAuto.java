package org.firstinspires.ftc.teamcode.Auto.red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
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

    //---------------CAROUSEL AUTO POS-----------
    Pose2d startRedCarouselPose = new Pose2d(-40.085, -63.54, Math.toRadians(270.0));
    Pose2d carouselPose = new Pose2d(-54.0, -59.0, radians(270.0));

    enum RedCarouselShippingHub {
        FIRST_LEVEL(new Pose2d(-27.0, -24.0, Math.toRadians(170)), Lifter.LEVEL.FIRST),

        SECOND_LEVEL(new Pose2d(-26.95, -24.0, Math.toRadians(175)), Lifter.LEVEL.SECOND),

        THIRD_LEVEL(new Pose2d(-26.7, -24.0, Math.toRadians(172)), Lifter.LEVEL.THIRD);

        Pose2d goTo;
        Lifter.LEVEL level;

        RedCarouselShippingHub(Pose2d goTo, Lifter.LEVEL level) {
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

        Thread updater = new Thread(new RedCarouselAuto.Updater());

        initWebcam();
        sleep(1000);
        cameraThread = new CameraThread(webcam);
        Thread cameraRunner = new Thread(cameraThread);
        cameraRunner.start();

        cameraThread.setState(CameraThread.CAMERA_STATE.INIT);
        sleep(1000);
        cameraThread.setState(CameraThread.CAMERA_STATE.STREAM);

        drive = new MecanumDriveImpl(hardwareMap);

        TrajectorySequence carousel = toCarousel(startRedCarouselPose);
        TrajectorySequence preload = preload(carousel.end(), RedCarouselShippingHub.THIRD_LEVEL);
        TrajectorySequence deliverDuck = deliverDuck(preload.end());

        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        //detect go brr
        result = CameraThread.getResult();
        telemetry.addData("Result", result);
        telemetry.update();

        cameraThread.setState(CameraThread.CAMERA_STATE.KILL);

        switch (result) {
            case FIRST:
                preload = preload(carousel.end(), RedCarouselShippingHub.FIRST_LEVEL);
                break;
            case SECOND:
                preload = preload(carousel.end(), RedCarouselShippingHub.SECOND_LEVEL);
                break;
            default:
                preload = preload(carousel.end(), RedCarouselShippingHub.THIRD_LEVEL);
                break;

        }

        updater.start(); //start calling update for intake and lifter

        //start action
        drive.setPoseEstimate(startRedCarouselPose);
        drive.followTrajectorySequence(carousel);
        drive.followTrajectorySequence(preload);
        drive.followTrajectorySequence(deliverDuck);
        lifter.closeBox();
        lifter.goToPosition(0,Lifter.LEVEL.DOWN.ticks);
        sleep(5000);

    }

    //-----------TRAJECTORIES-----------

    TrajectorySequence toCarousel(Pose2d startPose) {
        return drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    duckMechanism.startSpin();
                })
                .lineToLinearHeading(carouselPose)
                .waitSeconds(0.8)
                .build();
    }

    TrajectorySequence preload(Pose2d lastPose, RedCarouselShippingHub level) {
        return drive.trajectorySequenceBuilder(lastPose)
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    lifter.goToPosition(100, level.level.ticks);
                    lifter.intermediateBoxPosition(300);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.2, () -> {
                    lifter.depositMineral(0);
                    lifter.goToPosition(1000, Lifter.LEVEL.DOWN.ticks);
                })

                .UNSTABLE_addTemporalMarkerOffset(3.5, () -> {

                    intake.startIntakeVel(600);
                    intake.lowerIntake();
                    intake.releaseElements();
                    duckMechanism.stopSpin();

                })

                .setVelConstraint(new TranslationalVelocityConstraint(30.0))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-55.0, -35.0, Math.toRadians(270)), Math.toRadians(90.0))
                .splineToSplineHeading(level.goTo, Math.toRadians(360.0))
                .waitSeconds(0.1)
                .setReversed(false)

                .splineToSplineHeading(new Pose2d(-57.7, -35.0, Math.toRadians(270)), Math.toRadians(270.0))
                .splineToSplineHeading(new Pose2d(-53.7, -48.0, Math.toRadians(300)), Math.toRadians(300))

                .build();

    }

    TrajectorySequence deliverDuck(Pose2d lastPose) {
        return drive.trajectorySequenceBuilder(lastPose)
                .setVelConstraint(new TranslationalVelocityConstraint(20))
                .lineToLinearHeading(new Pose2d(-20.7, -54.3, Math.toRadians(250)))

                .setVelConstraint(new TranslationalVelocityConstraint(8))
                .setAccelConstraint(new ProfileAccelerationConstraint(15.0))
                .lineToLinearHeading(new Pose2d(-50.0, -57.5, Math.toRadians(230.0)))
                .lineToLinearHeading(new Pose2d(-53.0, -55.3, Math.toRadians(220.0)))

                .resetAccelConstraint()
                .resetVelConstraint()

                .setReversed(true)
                .resetVelConstraint()
                .lineToLinearHeading(new Pose2d(-60.0, -24.0, Math.toRadians(270)))
                .lineToLinearHeading(RedCarouselShippingHub.THIRD_LEVEL.goTo)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lifter.goToPosition(0, Lifter.LEVEL.THIRD.ticks);
                    lifter.intermediateBoxPosition(300);
                    lifter.depositMineral(500);//600
                    lifter.goToPosition(1000, Lifter.LEVEL.DOWN.ticks);
                    intake.raiseIntake();
                    intake.stopIntake();
                })
                .waitSeconds(0.5)
                .setReversed(false)

                .splineToSplineHeading(new Pose2d(-59.0, -37.3, Math.toRadians(270.0)), Math.toRadians(270.0))

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

}
