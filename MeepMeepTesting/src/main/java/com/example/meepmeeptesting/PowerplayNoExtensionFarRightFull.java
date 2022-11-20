package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class PowerplayNoExtensionFarRightFull {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(68, 68, Math.toRadians(248), Math.toRadians(180), 13.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 60, Math.toRadians(-90)))
                                //preloaded cone
                                .forward(8)
                                .splineToSplineHeading(new Pose2d(-8, 32, Math.toRadians(135)), Math.toRadians(0))
                                .setReversed(false)
                                .waitSeconds(.75)
                                .splineToSplineHeading(new Pose2d(-36, 24, Math.toRadians(-90)), Math.toRadians(-90))
                                .splineToSplineHeading(new Pose2d(-58, 12, Math.toRadians(180)), Math.toRadians(-190))
                                .waitSeconds(.75)
                                .setReversed(true)
                                //2nd cone
                                .splineToSplineHeading(new Pose2d(-8, 20, Math.toRadians(205)), Math.toRadians(25))
                                .waitSeconds(.75)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(-58, 12, Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(.75)
                                .setReversed(true)
                                //3rd cone
                                .splineToSplineHeading(new Pose2d(-8, 20, Math.toRadians(205)), Math.toRadians(25))
                                .waitSeconds(.75)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(-58, 12, Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(.75)
                                .setReversed(true)
                                //4th cone
                                .splineToSplineHeading(new Pose2d(-8, 20, Math.toRadians(205)), Math.toRadians(25))
                                .waitSeconds(.75)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(-58, 12, Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(.75)
                                .setReversed(true)
                                //5th cone
                                .splineToSplineHeading(new Pose2d(-8, 20, Math.toRadians(205)), Math.toRadians(25))
                                .waitSeconds(.75)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(-58, 12, Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(.75)
                                .setReversed(true)
                                //6th cone
                                .splineToSplineHeading(new Pose2d(-8, 20, Math.toRadians(205)), Math.toRadians(25))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}