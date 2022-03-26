package com.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class RedMeepMeep {

    //--------------WAREHOUSE AUTO POS-----------
    static Pose2d startRedWareHousePose = new Pose2d(7.915, -63.54, Math.toRadians(270.0)); //x:11.6
    static Pose2d redWShippingHubPose = new Pose2d(-5.83, -44.5, Math.toRadians(280.0));//x:-9.0
    static Pose2d inRedWarehousePose = new Pose2d(47.0, -67.3, Math.toRadians(0.0));

    //---------------CAROUSEL AUTO POS-----------
    static Pose2d startRedCarouselPose = new Pose2d(-40.085, -63.54, Math.toRadians(270.0));
    static Pose2d carouselPose = new Pose2d(-54.3, -59.0, radians(270.0));
    static Pose2d redCThirdLevelPose = new Pose2d(-27.0, -24.0, Math.toRadians(175));

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(650);

        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(57, 57, Math.toRadians(180), Math.toRadians(180), 10.2362)
                .setDimensions(12.59, 16.14).build();

        //myFirstBot.followTrajectorySequence(carousel(myFirstBot));

        myFirstBot.followTrajectorySequence(warehouse(myFirstBot, startRedWareHousePose, 2, 0));

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

    public static TrajectorySequence warehouse(RoadRunnerBotEntity bot, Pose2d initialPose, double xAdd, double yAdd) {
        DriveShim drive = bot.getDrive();

        return drive.trajectorySequenceBuilder(initialPose)
                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                    /*lifter.goToPosition(100, Lifter.LEVEL.THIRD.ticks);
                    lifter.intermediateBoxPosition(300);*/
                })
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {
                    /*lifter.depositMineral(0);
                    lifter.goToPosition(1000, Lifter.LEVEL.DOWN.ticks);*/
                })
                .lineToLinearHeading(redWShippingHubPose)

                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    /*intake.startIntake();
                    intake.lowerIntake();*/
                })
                /*.addTemporalMarker(0.9, () -> {
                 *//*intake.startIntake();
                    intake.lowerIntake();*//*
                })*/

                .lineToSplineHeading(new Pose2d(8.0, -61.5, radians(0))) //good one!
                .splineToLinearHeading(new Pose2d(43 + xAdd, -67.0 + yAdd, radians(0.0)), radians(25.0))
                .waitSeconds(0.1)

                //deliver freight
                .resetVelConstraint()
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                           /* intake.raiseIntake();
                            intake.stopIntake();*/
                        }
                )

                .splineToLinearHeading(new Pose2d(7.5, -67.2, radians(0.0)), radians(140.0))


                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    lifter.goToPosition(100, Lifter.LEVEL.THIRD.ticks);
//                    lifter.intermediateBoxPosition(300);
                })


                //GO TO SHIPPING HUB
                .splineToSplineHeading(redWShippingHubPose, Math.toRadians(115.0))

                .addTemporalMarker(() -> {
//                    lifter.depositMineral(0);
//                    lifter.goToPosition(1000, Lifter.LEVEL.DOWN.ticks);
                })
                .setReversed(false)
                .resetVelConstraint()

//                //park
//                .splineToSplineHeading(new Pose2d(8.0, -67.5, radians(0)),Math.toRadians(0))
//                .splineToSplineHeading(new Pose2d(43.0, -68.0, radians(0)),Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(7.0, -67.0, radians(0)),Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(43.0, -68.0, radians(0)),Math.toRadians(10))//-320

                .build();
    }

    public static TrajectorySequence carousel(RoadRunnerBotEntity bot) {
        DriveShim drive = bot.getDrive();

        return drive.trajectorySequenceBuilder(startRedCarouselPose)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    //duckMechanism.startSpin();
                })
                .lineToLinearHeading(carouselPose)
                .waitSeconds(0.8)

                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
//                    lifter.goToPosition(100, level.level.ticks);
//                    lifter.intermediateBoxPosition(300);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.2, () -> {
//                    lifter.depositMineral(0);
//                    lifter.goToPosition(1000, Lifter.LEVEL.DOWN.ticks);
                })

                .UNSTABLE_addTemporalMarkerOffset(3.5, () -> {

//                    intake.startIntakeVel(600);
//                    intake.lowerIntake();
//                    intake.releaseElements();
//                    duckMechanism.stopSpin();

                })

                .setVelConstraint(new TranslationalVelocityConstraint(30.0))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-55.0, -35.0, Math.toRadians(270)), Math.toRadians(90.0))
                .splineToSplineHeading(redCThirdLevelPose, Math.toRadians(360.0))
                .setReversed(false)

                .splineToSplineHeading(new Pose2d(-55.7, -35.0, Math.toRadians(270)), Math.toRadians(270.0))
                .splineToSplineHeading(new Pose2d(-53.7, -47.0, Math.toRadians(300)), Math.toRadians(300))
//---------deliverDuck
                .setVelConstraint(new TranslationalVelocityConstraint(20))
                .lineToLinearHeading(new Pose2d(-20.7, -54.0, Math.toRadians(255)))
                .setVelConstraint(new TranslationalVelocityConstraint(8))
                .setAccelConstraint(new ProfileAccelerationConstraint(15.0))
                .lineToLinearHeading(new Pose2d(-50.0, -57.0, Math.toRadians(240.0)))
                .lineToLinearHeading(new Pose2d(-56.0, -55.0, Math.toRadians(235.0)))

                .resetVelConstraint()
                .resetAccelConstraint()

                .setReversed(true)
                .resetVelConstraint()
                .lineToLinearHeading(new Pose2d(-61.0, -20.0, Math.toRadians(270)))
                .lineToLinearHeading(redCThirdLevelPose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    lifter.goToPosition(0, Lifter.LEVEL.THIRD.ticks);
//                    lifter.intermediateBoxPosition(300);
//                    lifter.depositMineral(600);
//                    intake.raiseIntake();
//                    intake.stopIntake();
                })
                .waitSeconds(0.5)
                .setReversed(false)

                .splineToSplineHeading(new Pose2d(-59.5, -37.3, Math.toRadians(270.0)), Math.toRadians(270.0))


                .build();
    }


    static double radians(double deg) {
        return Math.toRadians(deg);
    }

}
