package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class PowerplayFarRightBenTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                //--->hopefully this is more optimized for points<---
                .setConstraints(60, 68, Math.toRadians(248), Math.toRadians(180), 13.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 60, Math.toRadians(-90)))
                                //preloaded cone - low 1: 6pts
                                .splineToSplineHeading(new Pose2d(-32, 52, Math.toRadians(155)), Math.toRadians(0))
                                .waitSeconds(.75)
                                .lineToSplineHeading(new Pose2d(-36,36, Math.toRadians(180)))
                                .splineToSplineHeading(new Pose2d(-36, 20, Math.toRadians(180)), Math.toRadians(-90))
                                .splineToLinearHeading(new Pose2d(-58, 12, Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(.75)
                                //2nd cone - high #1: 8pts
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-30, 8, Math.toRadians(135)), Math.toRadians(-25))
                                .waitSeconds(.75)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(-58, 12, Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(.75)
                                //3rd cone - high #2: 8 pts
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-8, 20, Math.toRadians(205)), Math.toRadians(25))
                                .waitSeconds(.75)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(-58, 12, Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(.75)
                                //4th cone - medium 1: 7 pts
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-31, 15, Math.toRadians(-135)), Math.toRadians(25))
                                .waitSeconds(.75)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(-58, 12, Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(.75)
                                //5th cone - low 2: 6 pts
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-54, 17, Math.toRadians(-135)), Math.toRadians(90))
                                .waitSeconds(.75)
                                .setReversed(false)
                                .splineToLinearHeading(new Pose2d(-58, 12, Math.toRadians(180)), Math.toRadians(-90))
                                .waitSeconds(.75)
                                //6th cone - high #3: 5 pts
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-8, 20, Math.toRadians(205)), Math.toRadians(25))
                                .setReversed(false)
                                .build()
                        //max total pts: 40. This assumes each junction remains in our control
                        //minimum points: 25. This assumes each junction is in enemy control
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}