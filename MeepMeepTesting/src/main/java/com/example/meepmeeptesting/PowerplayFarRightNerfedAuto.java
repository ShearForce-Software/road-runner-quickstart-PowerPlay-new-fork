package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class PowerplayFarRightNerfedAuto {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                //--->hopefully this is more optimized for points<---
                .setConstraints(60, 68, Math.toRadians(248), Math.toRadians(180), 13.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 60, Math.toRadians(-90)))
                                //preloaded cone - high: 8pts
                                .forward(20)
                                .lineToSplineHeading(new Pose2d(-36, 32, Math.toRadians(90)))
                                .waitSeconds(1)//raising to high
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-29, 7, Math.toRadians(135)), Math.toRadians(-25))
                                .waitSeconds(.75)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(-48, 12, Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(1)//lower slides
                                .splineToSplineHeading(new Pose2d(-58, 12, Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(.75)
                                //2nd cone - high #2: 8 pts
                                .setReversed(true)
                                .forward(-12)
                                .waitSeconds(1)//raising to high
                                .splineToSplineHeading(new Pose2d(-8, 20, Math.toRadians(205)), Math.toRadians(25))
                                .waitSeconds(.75)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(-48, 12, Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(1)//lower slides
                                .splineToSplineHeading(new Pose2d(-58, 12, Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(.75)
                                //3rd cone - medium #1: 7pts
                                .setReversed(true)
                                .forward(-12)
                                .waitSeconds(1)//raising to medium
                                .splineToSplineHeading(new Pose2d(-30, 16, Math.toRadians(-135)), Math.toRadians(25))
                                .waitSeconds(.75)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(-48, 12, Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(1)//lower slides
                                .splineToSplineHeading(new Pose2d(-58, 12, Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(.75)
                                //4th cone - low #1: 6pts
                                .setReversed(true)
                                .lineToSplineHeading(new Pose2d(-54, 12, Math.toRadians(-90)))
                                .waitSeconds(1)//raising to low
                                .lineTo(new Vector2d(-47, 12))
                                .forward(-2)
                                .waitSeconds(.75)
                                //parking prep
                                .forward(2)
                                .setReversed(false)
                                //spot to start park (-47,12, H:-90)
                                /*
                                .lineTo(new Vector2d(-12, 12))//park 1
                                .lineTo(new Vector2d(-36, 12))//park 2
                                .lineTo(new Vector2d(-58, 12))//park 3
                                 */
                                .build()
                        //max total cone pts: 29. This assumes each junction remains in our control
                        //minimum cone points: 17. This assumes each junction is in enemy control
                        //+20 for parking
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}