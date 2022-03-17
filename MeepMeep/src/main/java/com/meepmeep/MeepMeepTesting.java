package com.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class MeepMeepTesting {

    static Pose2d startWareHousePose = new Pose2d(7.915, -63.54, Math.toRadians(270.0)); //x:11.6
    static Pose2d startCarouselPose = new Pose2d(-40.085, -63.54, Math.toRadians(270.0));
    static Pose2d shippingHubPose = new Pose2d(-5.85, -45.0, Math.toRadians(280.0));//x:-9.0
    static Pose2d inWarehousePose = new Pose2d(47.0, -66.3, Math.toRadians(0.0));


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(650);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(55, 55, Math.toRadians(180), Math.toRadians(180), 10.2362)
                .setDimensions(12.59, 16.14).build();
        myBot.followTrajectorySequence(redWarehousePreload(myBot, 0.1,0.1));

        //myBot.followTrajectorySequence(cycles(myBot, 0.1,0.1));

//            .followTrajectorySequence(drive ->
//                    drive.trajectorySequenceBuilder(startPose)
//
//                            .lineToLinearHeading(shippingHubPose) //preload
//                            .waitSeconds(0.4)
//
//                            //-START OF CYCLE
//                            //collecting
//                            //.lineToLinearHeading(new Pose2d(6.84, -69.0, radians(0.0)))
//                            //.splineToLinearHeading(new Pose2d(50.0, -69.0, radians(0.0)), 0.0)
//                            .splineToSplineHeading(new Pose2d(12.0, -68.0, radians(0.0)), radians(0.0))
//                            .splineToSplineHeading(new Pose2d(50.0, -69.0, radians(0.0)), radians(0.0))
//
//                            .waitSeconds(0.1)
//
//                            //deliver freight
//                            .setReversed(true)
//                            .splineToSplineHeading(new Pose2d(23.3, -69.0, radians(0.0)), radians(180.0))
//                            //.splineToSplineHeading(new Pose2d(9.0, -72.0, radians(355.0)), radians(175.0))
//                            .resetVelConstraint()
//                            .splineToSplineHeading(shippingHubPose, radians(80.0))
//                            .waitSeconds(0.5)
//                            .setReversed(false)
//                            .resetVelConstraint()
//                            //-----END OF CYCLE----
//
//                            .build()
//            );

        meepMeep.setBackground(com.noahbres.meepmeep.MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    public static TrajectorySequence redWarehousePreload(RoadRunnerBotEntity bot, double xModif, double yModif) {
        DriveShim drive = bot.getDrive();
        return drive.trajectorySequenceBuilder(startWareHousePose)
                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                   /* lifter.goToPosition(100, Lifter.LEVEL.THIRD.ticks);
                    lifter.intermediateBoxPosition(300);*/
                })
                .lineToLinearHeading(shippingHubPose) //preload
                .addTemporalMarker(() -> {
                    /*lifter.depositMineral(0);
                    lifter.goToPosition(1000, Lifter.LEVEL.DOWN.ticks);*/
                })
                .splineToSplineHeading(new Pose2d(23.3, -66.3, radians(0.0)), radians(0.0))//347
                .splineToSplineHeading(new Pose2d(47.0, -66.3), radians(0.0))

                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                   /*
                   intake...
                    */
                })

                //collect freight
                .lineToLinearHeading(new Pose2d(47.0 + xModif, -66.3 + yModif, radians(0)))

                //deliver freight
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(32.0, -66.3, radians(0.0)), radians(180.0))
                .addTemporalMarker(() -> {
                            //intake up
                        }
                )
                .splineToSplineHeading(shippingHubPose, radians(90.0))
                .waitSeconds(0.5)
                .setReversed(false)
                .resetVelConstraint()
                //-----END OF CYCLE----

                .splineToSplineHeading(new Pose2d(23.3, -66.3, radians(0.0)), radians(0.0))//347
                .splineToSplineHeading(inWarehousePose, radians(0.0))

                .build();
    }


    static double radians(double deg) {
        return Math.toRadians(deg);
    }

}
