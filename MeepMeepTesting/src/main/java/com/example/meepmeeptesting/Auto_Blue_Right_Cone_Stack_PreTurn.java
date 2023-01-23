package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
//this is a test

public class Auto_Blue_Right_Cone_Stack_PreTurn {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        Vector2d junctionPos = new Vector2d(28.8,-7.8);
        Pose2d stackPos = new Pose2d(63, -12, Math.toRadians(0));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12.0)
                .setDimensions(16,16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(36, -64.5, Math.toRadians(90)))
                                // /*
                                //first cone
                                .strafeTo(new Vector2d(36,-56))
                                .splineToSplineHeading(new Pose2d(36, -28, 0), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(36, -24), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(36, -16.5, Math.toRadians(-45)), Math.toRadians(90))
                                .splineToConstantHeading(junctionPos, Math.toRadians(135))
                                .waitSeconds(0.25)
                                //stack
                                .splineToSplineHeading(new Pose2d(38, -12, Math.toRadians(0)), Math.toRadians(0))
                                .splineToLinearHeading(stackPos, Math.toRadians(0))
                                .waitSeconds(.75)
                                .setReversed(true)
                                ////2nd cone
                                .strafeTo(new Vector2d(44,-12))
                                .splineToSplineHeading(new Pose2d(36, -12, Math.toRadians(-45)), Math.toRadians(180))
                                .splineToConstantHeading(junctionPos, Math.toRadians(135))
                                .waitSeconds(.25)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(38, -12, Math.toRadians(0)), Math.toRadians(0))
                                .splineToLinearHeading(stackPos, Math.toRadians(0))
                                .waitSeconds(.75)
                                .setReversed(true)
                                //3rd cone
                                .strafeTo(new Vector2d(44,-12))
                                .splineToSplineHeading(new Pose2d(36, -12, Math.toRadians(-45)), Math.toRadians(180))
                                .splineToConstantHeading(junctionPos, Math.toRadians(135))
                                .waitSeconds(.25)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(38, -12, Math.toRadians(0)), Math.toRadians(0))
                                .splineToLinearHeading(stackPos, Math.toRadians(0))
                                .waitSeconds(.75)
                                .setReversed(true)
                                //4th cone
                                .strafeTo(new Vector2d(44,-12))
                                .splineToSplineHeading(new Pose2d(36, -12, Math.toRadians(-45)), Math.toRadians(180))
                                .splineToConstantHeading(junctionPos, Math.toRadians(135))
                                .waitSeconds(.25)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(38, -12, Math.toRadians(0)), Math.toRadians(0))
                                .splineToLinearHeading(stackPos, Math.toRadians(0))
                                .waitSeconds(.75)
                                .setReversed(true)
                                //5th cone
                                .strafeTo(new Vector2d(44,-12))
                                .splineToSplineHeading(new Pose2d(36, -12, Math.toRadians(-45)), Math.toRadians(180))
                                .splineToConstantHeading(junctionPos, Math.toRadians(135))
                                .waitSeconds(.25)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(38, -12, Math.toRadians(0)), Math.toRadians(0))
                                .splineToLinearHeading(stackPos, Math.toRadians(0))
                                .waitSeconds(.75)
                                .setReversed(true)
                                //6th cone
                                .strafeTo(new Vector2d(44,-12))
                                .splineToSplineHeading(new Pose2d(36, -12, Math.toRadians(-45)), Math.toRadians(180))
                                .splineToConstantHeading(junctionPos, Math.toRadians(135))
                                .waitSeconds(.25)
                                .setReversed(false)
                                //>>>>>>>>>>>parking<<<<<<<<<<<<<
                                //park 2 only
                                //.splineToLinearHeading(new Pose2d(35.5, -12, Math.toRadians(-90)), Math.toRadians(-90))
                                //for park 1 & 3
                                .splineToSplineHeading(new Pose2d(36, -24, Math.toRadians(-90)), Math.toRadians(-90))
                                //park 1
                                //.splineToConstantHeading(new Vector2d(24, -36), Math.toRadians(180))
                                //.splineToConstantHeading(new Vector2d(12, -36), Math.toRadians(180))
                                //park 3
                                .splineToConstantHeading(new Vector2d(48, -36), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(60, -36), Math.toRadians(0))

                                // */
                                // /\
                                //CHRIS STUFF
                                //OLD STUFF
                                // \/
                                 /*
                                //29.27
                                .forward(4)
                                .splineToSplineHeading(new Pose2d(36, -34, Math.toRadians(-91)), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(36, -25, Math.toRadians(-45)), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(29,-8), Math.toRadians(135))
                                .waitSeconds(.75)
                                .setReversed(false)
                                //.splineToSplineHeading(linePos, Math.toRadians(0))
                                // slow down this portion of the trajectory
                                .splineToLinearHeading(stackPos, Math.toRadians(0))
                                .waitSeconds(.75)
                                .setReversed(true)
                                //2nd cone
                                .splineToSplineHeading(new Pose2d(52, -13,Math.toRadians(-25)), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(29.8, -5), Math.toRadians(155))
                                .waitSeconds(.75)
                                .setReversed(false)
                                .splineToLinearHeading(stackPos, Math.toRadians(0))
                                .waitSeconds(.75)
                                .setReversed(true)
                                //3rd cone
                                .splineToSplineHeading(new Pose2d(52, -13,Math.toRadians(-25)), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(29.8, -5), Math.toRadians(155))
                                .waitSeconds(.75)
                                .setReversed(false)
                                .splineToLinearHeading(stackPos, Math.toRadians(0))
                                .waitSeconds(.75)
                                .setReversed(true)
                                //4th cone
                                .splineToSplineHeading(new Pose2d(52, -13,Math.toRadians(-25)), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(31,-4), Math.toRadians(155))
                                .waitSeconds(.75)
                                .setReversed(false)
                                .splineToLinearHeading(stackPos, Math.toRadians(0))
                                .waitSeconds(.75)
                                .setReversed(true)
                                //5th cone
                                .splineToSplineHeading(new Pose2d(52, -13,Math.toRadians(-25)), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(31,-4), Math.toRadians(155))
                                .waitSeconds(.75)
                                .setReversed(false)
                                .splineToLinearHeading(stackPos, Math.toRadians(0))
                                .waitSeconds(.75)
                                .setReversed(true)
                                //6th cone
                                .splineToSplineHeading(new Pose2d(52, -13,Math.toRadians(-25)), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(31,-4), Math.toRadians(155))
                                .waitSeconds(.75)
                                //>>>>>>>>>>>park<<<<<<<<<<<<<
                                .splineToLinearHeading(new Pose2d(31, -12, Math.toRadians(-90)), Math.toRadians(180))
                                .strafeTo(new Vector2d(12,-12))
                                 */
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}