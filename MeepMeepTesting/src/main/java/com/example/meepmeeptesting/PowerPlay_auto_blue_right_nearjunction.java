package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class PowerPlay_auto_blue_right_nearjunction {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(65, 65, Math.toRadians(200), Math.toRadians(200), 12.0)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 60, Math.toRadians(-90)))
                                //.forward(6)
                                //.turn(Math.toRadians(180))
                                //.splineToLinearHeading(new Pose2d(-50, 12, Math.toRadians(0)), Math.toRadians(-180))
                                //1st cone
                                //.setReversed(true)
                                .forward(12)
                                .setReversed(false)
                                .splineToLinearHeading(new Pose2d(-31, 7, Math.toRadians(135)), Math.toRadians(-45))
                                .waitSeconds(.75)
                                .setReversed(false)
                                .splineToLinearHeading(new Pose2d(-58, 12, Math.toRadians(-180)), Math.toRadians(-180))
                                .waitSeconds(.75)
                                .setReversed(true)
                                //2nd cone
                                .splineToLinearHeading(new Pose2d(-31, 7, Math.toRadians(135)), Math.toRadians(-45))
                                .waitSeconds(.75)
                                .setReversed(false)
                                .splineToLinearHeading(new Pose2d(-58, 12, Math.toRadians(-180)), Math.toRadians(-180))
                                .waitSeconds(.75)
                                .setReversed(true)
                                //3rd cone
                                .splineToLinearHeading(new Pose2d(-31, 7, Math.toRadians(135)), Math.toRadians(-45))
                                .waitSeconds(.75)
                                .setReversed(false)
                                .splineToLinearHeading(new Pose2d(-58, 12, Math.toRadians(-180)), Math.toRadians(-180))
                                .waitSeconds(.75)
                                .setReversed(true)
                                //4th cone
                                .splineToLinearHeading(new Pose2d(-31, 7, Math.toRadians(135)), Math.toRadians(-45))
                                .waitSeconds(.75)
                                .setReversed(false)
                                .splineToLinearHeading(new Pose2d(-58, 12, Math.toRadians(-180)), Math.toRadians(-180))
                                .waitSeconds(.75)
                                .setReversed(true)
                                //5th cone
                                .splineToLinearHeading(new Pose2d(-31, 7, Math.toRadians(135)), Math.toRadians(-45))
                                .waitSeconds(.75)
                                .setReversed(false)
                                .splineToLinearHeading(new Pose2d(-58, 12, Math.toRadians(-180)), Math.toRadians(-180))
                                .waitSeconds(.75)
                                .setReversed(true)
                                //6th cone
                                .splineToLinearHeading(new Pose2d(-31, 7, Math.toRadians(135)), Math.toRadians(-45))
                                .waitSeconds(.75)
                                .setReversed(false)
                                .splineToLinearHeading(new Pose2d(-58, 12, Math.toRadians(-180)), Math.toRadians(-180))
                                .back(48)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}