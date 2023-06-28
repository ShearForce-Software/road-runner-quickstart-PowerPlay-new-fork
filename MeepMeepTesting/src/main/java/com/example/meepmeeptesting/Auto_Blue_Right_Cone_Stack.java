package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;
//this is a test

public class Auto_Blue_Right_Cone_Stack {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        Pose2d junctionPos = new Pose2d(29,-8, Math.toRadians(-45));
        Pose2d stackPos = new Pose2d(60, -13.5, Math.toRadians(0));
        Pose2d linePos = new Pose2d(40, -13.5, Math.toRadians(0));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12.0)
                .setDimensions(16,16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(36, -64.5, Math.toRadians(90)))
                                .strafeTo(new Vector2d(36, -42))
                                //.forward(12.5)
                                .splineToSplineHeading(new Pose2d(36, -25, Math.toRadians(-90)), Math.toRadians(90))

                                .splineToSplineHeading(junctionPos, Math.toRadians(135))
                                .setReversed(false)
                                .splineToSplineHeading(linePos, Math.toRadians(0))
                                // slow down this portion of the trajectory
                                .splineToLinearHeading(stackPos, Math.toRadians(0))
                                .setReversed(true)
                                //2nd cone
                                .splineToSplineHeading(linePos, Math.toRadians(180))
                                .splineToSplineHeading(junctionPos, Math.toRadians(135))
                                .waitSeconds(.75)
                                .setReversed(false)
                                .splineToSplineHeading(linePos, Math.toRadians(0))
                                .splineToLinearHeading(stackPos, Math.toRadians(0))
                                .waitSeconds(.75)
                                .setReversed(true)
                                //3rd cone
                                .splineToSplineHeading(linePos, Math.toRadians(180))
                                .splineToSplineHeading(junctionPos, Math.toRadians(135))
                                .waitSeconds(.75)
                                .setReversed(false)
                                .splineToSplineHeading(linePos, Math.toRadians(0))
                                .splineToLinearHeading(stackPos, Math.toRadians(0))
                                .waitSeconds(.75)
                                .setReversed(true)
                                //4th cone
                                .splineToSplineHeading(linePos, Math.toRadians(180))
                                .splineToSplineHeading(junctionPos, Math.toRadians(135))
                                .waitSeconds(.75)
                                .setReversed(false)
                                .splineToSplineHeading(linePos, Math.toRadians(0))
                                .splineToLinearHeading(stackPos, Math.toRadians(0))
                                .waitSeconds(.75)
                                .setReversed(true)
                                //5th cone
                                .splineToSplineHeading(linePos, Math.toRadians(180))
                                .splineToSplineHeading(junctionPos, Math.toRadians(135))
                                .waitSeconds(.75)
                                .setReversed(false)
                                .splineToSplineHeading(linePos, Math.toRadians(0))
                                .splineToLinearHeading(stackPos, Math.toRadians(0))
                                .waitSeconds(.75)
                                .setReversed(true)
                                //6th cone
                                .splineToSplineHeading(linePos, Math.toRadians(180))
                                .splineToSplineHeading(junctionPos, Math.toRadians(135))
                                .waitSeconds(.75)
                                .setReversed(false)
                                .splineToLinearHeading(stackPos, Math.toRadians(0))
                                //>>>>>>>>>>>park<<<<<<<<<<<<<
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