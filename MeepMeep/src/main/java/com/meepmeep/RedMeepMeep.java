package com.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class RedMeepMeep {

    static Pose2d startRedWareHousePose = new Pose2d(7.915, -63.54, Math.toRadians(270.0)); //x:11.6
    static Pose2d redWShippingHubPose = new Pose2d(-5.83, -44.5, Math.toRadians(280.0));//x:-9.0
    static Pose2d inRedWarehousePose = new Pose2d(47.0, -66.3, Math.toRadians(0.0));

    static Pose2d startRedCarouselPose = new Pose2d(-40.085, -63.54, Math.toRadians(270.0));
    static Pose2d redCShippingHubPose = new Pose2d(-27.0, -36.5, Math.toRadians(220.0));
    static Pose2d carouselPose = new Pose2d(-54.3, -59.0, radians(270.0));

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(650);

        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(55, 55, Math.toRadians(180), Math.toRadians(180), 10.2362)
                .setDimensions(12.59, 16.14).build();

        myFirstBot.followTrajectorySequence(preloadCarousel(myFirstBot));

        //myFirstBot.followTrajectorySequence(preloadWarehouse(myFirstBot));

       /* RoadRunnerBotEntity mySecondBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(55, 55, Math.toRadians(180), Math.toRadians(180), 10.2362)
                .setDimensions(12.59, 16.14).build();

        mySecondBot.followTrajectorySequence(cycles(mySecondBot, redWShippingHubPose, 0.1,0.1));*/


        meepMeep.setBackground(com.noahbres.meepmeep.MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myFirstBot)
                //.addEntity(mySecondBot)
                .start();
    }

    public static TrajectorySequence preloadWarehouse(RoadRunnerBotEntity bot) {
        DriveShim drive = bot.getDrive();

        return drive.trajectorySequenceBuilder(startRedWareHousePose)
                //PRELOAD
                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                    /*lifter.goToPosition(100, Lifter.LEVEL.THIRD.ticks);
                    lifter.intermediateBoxPosition(300);*/
                })
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {
                    /*lifter.depositMineral(0);
                    lifter.goToPosition(1000, Lifter.LEVEL.DOWN.ticks);*/
                })
                .lineToLinearHeading(redWShippingHubPose)
                .build();
    }

    public static TrajectorySequence preloadCarousel(RoadRunnerBotEntity bot) {
        DriveShim drive = bot.getDrive();

        return drive.trajectorySequenceBuilder(startRedCarouselPose)
                //DELIVER PRELOAD
                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                    /*lifter.goToPosition(100, Lifter.LEVEL.THIRD.ticks);
                    lifter.intermediateBoxPosition(300);*/
                })
                .UNSTABLE_addTemporalMarkerOffset(0.85, () -> {
                    /*lifter.depositMineral(0);
                    lifter.goToPosition(1000, Lifter.LEVEL.DOWN.ticks);*/
                })

                .lineToLinearHeading(redCShippingHubPose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //duckMechanism.startSpin()
                })

                //spin DUCK
                .lineToLinearHeading(carouselPose)
                .waitSeconds(0.8)

                //COLLECT DUCK
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{
                    /*
                    intake.startIntake();
                    intake.lowerIntake();*/

                })
                .setVelConstraint(new TranslationalVelocityConstraint(15.0))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-27.8, -47.0, Math.toRadians(300.0)),Math.toRadians(10.0))
                .setVelConstraint(new TranslationalVelocityConstraint(5.0))
                .setReversed(false)
//-28.5
                .splineToSplineHeading(new Pose2d(-35.5, -56.5, Math.toRadians(270.0)), Math.toRadians(200.0))//265
                .resetAccelConstraint()
/*
                .splineToSplineHeading(new Pose2d(-40.5, -50.5, Math.toRadians(270.0)), Math.toRadians(45.0))//265

                .setVelConstraint(new TranslationalVelocityConstraint(5.0))
                .splineToSplineHeading(new Pose2d(-52.0, -56.5, Math.toRadians(270.0)), Math.toRadians(265.0))//265
                .resetAccelConstraint()
*/
                //DELIVER DUCK
                .build();
    }

    public static TrajectorySequence cycles(RoadRunnerBotEntity bot, Pose2d initialPose, double xAdd, double yAdd) {
        DriveShim drive = bot.getDrive();

        return drive.trajectorySequenceBuilder(initialPose)

                .addTemporalMarker(0.9, () -> {
                    /*intake.startIntake();
                    intake.lowerIntake();*/
                })

                .splineToSplineHeading(new Pose2d(23.3, -66.3 + yAdd, radians(0.0)), radians(0.0))//347

                //go to collect freight
                .splineToSplineHeading(new Pose2d(47.0 + xAdd, -66.3), radians(0.0))
                .waitSeconds(0.2)

                //deliver freight
                .resetVelConstraint()
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                          /*  intake.raiseIntake();
                            intake.stopIntake();*/
                        }
                )
                .splineToSplineHeading(new Pose2d(32.0, -66.5, radians(0.0)), radians(180.0))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                   /* lifter.goToPosition(100, Lifter.LEVEL.THIRD.ticks);
                    lifter.intermediateBoxPosition(300);*/
                })

                .splineToSplineHeading(redWShippingHubPose, radians(90.0))

                .addTemporalMarker(() -> {
                   /* lifter.depositMineral(0);
                    lifter.goToPosition(1000, Lifter.LEVEL.DOWN.ticks);*/
                })
                .waitSeconds(0.5)
                .setReversed(false)
                .resetVelConstraint()
                .build();
    }

    static double radians(double deg) {
        return Math.toRadians(deg);
    }

}
