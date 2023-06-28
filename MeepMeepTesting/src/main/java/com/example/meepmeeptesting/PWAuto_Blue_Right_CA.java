package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
//this is a test

public class PWAuto_Blue_Right_CA {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        Vector2d junctionPos = new Vector2d(24,-6);
        Pose2d stackPos = new Pose2d(63, -12, Math.toRadians(0));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12.0)
                .setDimensions(16,16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(36, -64.5, Math.toRadians(-90)))
                                // /*
                                //first cone
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(36, -14), Math.toRadians(90))
                                .splineToConstantHeading(junctionPos, Math.toRadians(90))
                                .waitSeconds(0.25)
                                //stack
                                .setReversed(false)
                                .splineToConstantHeading(new Vector2d(28,-12), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(48, -12, Math.toRadians(0)), Math.toRadians(0))
                                .splineToLinearHeading(stackPos, Math.toRadians(0))
                                .waitSeconds(.75)
                                .setReversed(true)
                                ////2nd cone
                                .strafeTo(new Vector2d(44,-12))
                                .splineToSplineHeading(new Pose2d(28, -12, Math.toRadians(-90)), Math.toRadians(180))
                                .splineToConstantHeading(junctionPos, Math.toRadians(90))
                                .waitSeconds(.25)
                                .setReversed(false)
                                .splineToConstantHeading(new Vector2d(28,-12), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(48, -12, Math.toRadians(0)), Math.toRadians(0))
                                .splineToLinearHeading(stackPos, Math.toRadians(0))
                                .waitSeconds(.75)
                                .setReversed(true)
                                //3rd cone
                                .strafeTo(new Vector2d(44,-12))
                                .splineToSplineHeading(new Pose2d(28, -12, Math.toRadians(-90)), Math.toRadians(180))
                                .splineToConstantHeading(junctionPos, Math.toRadians(90))
                                .waitSeconds(.25)
                                .setReversed(false)
                                .splineToConstantHeading(new Vector2d(28,-12), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(48, -12, Math.toRadians(0)), Math.toRadians(0))
                                .splineToLinearHeading(stackPos, Math.toRadians(0))
                                .waitSeconds(.75)
                                .setReversed(true)
                                //4th cone
                                .strafeTo(new Vector2d(44,-12))
                                .splineToSplineHeading(new Pose2d(28, -12, Math.toRadians(-90)), Math.toRadians(180))
                                .splineToConstantHeading(junctionPos, Math.toRadians(90))
                                .waitSeconds(.25)
                                .setReversed(false)
                                .splineToConstantHeading(new Vector2d(28,-12), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(48, -12, Math.toRadians(0)), Math.toRadians(0))
                                .splineToLinearHeading(stackPos, Math.toRadians(0))
                                .waitSeconds(.75)
                                .setReversed(true)
                                //5th cone
                                .strafeTo(new Vector2d(44,-12))
                                .splineToSplineHeading(new Pose2d(28, -12, Math.toRadians(-90)), Math.toRadians(180))
                                .splineToConstantHeading(junctionPos, Math.toRadians(90))
                                .waitSeconds(.25)
                                .setReversed(false)
                                .splineToConstantHeading(new Vector2d(28,-12), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(48, -12, Math.toRadians(0)), Math.toRadians(0))
                                .splineToLinearHeading(stackPos, Math.toRadians(0))
                                .waitSeconds(.75)
                                .setReversed(true)
                                //6th cone
                                .strafeTo(new Vector2d(44,-12))
                                .splineToSplineHeading(new Pose2d(28, -12, Math.toRadians(-90)), Math.toRadians(180))
                                .splineToConstantHeading(junctionPos, Math.toRadians(90))
                                .waitSeconds(.25)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}