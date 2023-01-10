package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Auto_Right_StackToHigh6_Edge {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        Pose2d OGjunctionPos = new Pose2d(29,-8, Math.toRadians(-45));
        Pose2d NEWjunctionPos = new Pose2d(31, -4, Math.toRadians(335));
        Pose2d stackPos = new Pose2d(60, -13.5, Math.toRadians(0));
        Pose2d linePos = new Pose2d(40, -13.5, Math.toRadians(0));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12.0)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(40, -64.5, Math.toRadians(90)))
                                .strafeTo(new Vector2d(35,-48))
                                //.splineToSplineHeading(new Pose2d(36, -48, Math.toRadians(90)), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(35, -24, Math.toRadians(-90)), Math.toRadians(90))
                                .splineToSplineHeading(OGjunctionPos, Math.toRadians(135))
                                .waitSeconds(.75)
                                .setReversed(false)
                                .splineToSplineHeading(linePos, Math.toRadians(0))

                                // slow down this portion of the trajectory
                                .splineToLinearHeading(stackPos, Math.toRadians(0))
                                .waitSeconds(.75)
                                .setReversed(true)
                                //2nd cone
                                .splineToSplineHeading(NEWjunctionPos, Math.toRadians(155))
                                .waitSeconds(.75)
                                .setReversed(false)
                                .splineToLinearHeading(stackPos, Math.toRadians(0))
                                .waitSeconds(.75)
                                .setReversed(true)
                                //3rd cone
                                .splineToSplineHeading(NEWjunctionPos, Math.toRadians(155))
                                .waitSeconds(.75)
                                .setReversed(false)
                                .splineToLinearHeading(stackPos, Math.toRadians(0))
                                .waitSeconds(.75)
                                .setReversed(true)
                                //4th cone
                                .splineToSplineHeading(NEWjunctionPos, Math.toRadians(155))
                                .waitSeconds(.75)
                                .setReversed(false)
                                .splineToLinearHeading(stackPos, Math.toRadians(0))
                                .waitSeconds(.75)
                                .setReversed(true)
                                //5th cone
                                .splineToSplineHeading(NEWjunctionPos, Math.toRadians(155))
                                .waitSeconds(.75)
                                .setReversed(false)
                                .splineToLinearHeading(stackPos, Math.toRadians(0))
                                .waitSeconds(.75)
                                .setReversed(true)
                                //6th cone
                                .splineToSplineHeading(NEWjunctionPos, Math.toRadians(155))
                                .waitSeconds(.75)
                                //>>>>>>>>>>>park<<<<<<<<<<<<<
                                .splineToLinearHeading(new Pose2d(31, -12, Math.toRadians(-90)), Math.toRadians(180))
                                .strafeTo(new Vector2d(12,-12))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}