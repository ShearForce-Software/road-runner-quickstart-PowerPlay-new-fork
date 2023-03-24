package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
//this is a test

public class Auto_Blue_Left_Cone_Stack {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        Pose2d startPose = new Pose2d(-36, -64.5, Math.toRadians(-90));
        Vector2d junctionPos = new Vector2d(-27,-7.4);
        Pose2d stackPos = new Pose2d(-63, -13.26, Math.toRadians(180));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12.0)
                .setDimensions(16,16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                //.strafeTo(new Vector2d(-36,-58))
                                //.splineToSplineHeading(new Pose2d(-36, -28, Math.toRadians(180)), Math.toRadians(90))
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(-36, -20), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(-36, -20, Math.toRadians(-135)), Math.toRadians(90))
                                .splineToConstantHeading(junctionPos, Math.toRadians(45))
                                .waitSeconds(0.25)
                                //stack
                                .splineToSplineHeading(new Pose2d(-38, -13.26, Math.toRadians(180)), Math.toRadians(180))
                                .splineToLinearHeading(stackPos, Math.toRadians(180))
                                .waitSeconds(.75)
                                .setReversed(true)
                                ////2nd cone
                                .strafeTo(new Vector2d(-44,-13.26))
                                .splineToSplineHeading(new Pose2d(-36, -13.26, Math.toRadians(-135)), Math.toRadians(0))
                                .splineToConstantHeading(junctionPos, Math.toRadians(45))
                                .waitSeconds(.25)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(-38, -13.26, Math.toRadians(180)), Math.toRadians(180))
                                .splineToLinearHeading(stackPos, Math.toRadians(180))
                                .waitSeconds(.75)
                                .setReversed(true)
                                //3rd cone
                                .strafeTo(new Vector2d(-44,-13.26))
                                .splineToSplineHeading(new Pose2d(-36, -13.26, Math.toRadians(-135)), Math.toRadians(0))
                                .splineToConstantHeading(junctionPos, Math.toRadians(45))
                                .waitSeconds(.25)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(-38, -13.26, Math.toRadians(180)), Math.toRadians(180))
                                .splineToLinearHeading(stackPos, Math.toRadians(180))
                                .waitSeconds(.75)
                                .setReversed(true)
                                //4th cone
                                .strafeTo(new Vector2d(-44,-13.26))
                                .splineToSplineHeading(new Pose2d(-36, -13.26, Math.toRadians(-135)), Math.toRadians(0))
                                .splineToConstantHeading(junctionPos, Math.toRadians(45))
                                .waitSeconds(.25)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(-38, -13.26, Math.toRadians(180)), Math.toRadians(180))
                                .splineToLinearHeading(stackPos, Math.toRadians(180))
                                .waitSeconds(.75)
                                .setReversed(true)
                                //5th cone
                                .strafeTo(new Vector2d(-44,-13.26))
                                .splineToSplineHeading(new Pose2d(-36, -13.26, Math.toRadians(-135)), Math.toRadians(0))
                                .splineToConstantHeading(junctionPos, Math.toRadians(45))
                                .waitSeconds(.25)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(-38, -13.26, Math.toRadians(180)), Math.toRadians(180))
                                .splineToLinearHeading(stackPos, Math.toRadians(180))
                                .waitSeconds(.75)
                                .setReversed(true)
                                //6th cone
                                .strafeTo(new Vector2d(-44,-13.26))
                                .splineToSplineHeading(new Pose2d(-36, -13.26, Math.toRadians(-135)), Math.toRadians(0))
                                .splineToConstantHeading(junctionPos, Math.toRadians(45))
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