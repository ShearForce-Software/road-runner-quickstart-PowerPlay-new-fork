package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class PowerPlay_Auto_Blue_Right_Marker_Test {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        Pose2d junctionPos = new Pose2d(-30,-2, Math.toRadians(-135));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12.0)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -60, Math.toRadians(90)))
                                .forward(12)
                                .splineToSplineHeading(new Pose2d(-36, -24, Math.toRadians(-90)), Math.toRadians(90))
                                .splineToSplineHeading(junctionPos, Math.toRadians(45))
                                .addTemporalMarker(0, () -> {
                                    // close claw
                                    // raise slide elevator
                                })
                                .addTemporalMarker(1.5, () -> {
                                    // swing arm to stow cone
                                })
                                .addTemporalMarker(1.65, () -> {
                                    // swing arm to high position

                                })
                                .addTemporalMarker(2.65, () -> {
                                    // rotate arm 180 degrees
                                    // straighten wrist
                                })

                                //.setReversed(true)
                                // slow down this portion of the trajectory
                                //.splineToLinearHeading(new Pose2d(-31, 7, Math.toRadians(135)), Math.toRadians(-45))
                                // chassis distance sensor to stop forward motion?
                                // chassis distance sensor to release cone

                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(-48, -12, Math.toRadians(-180)), Math.toRadians(-180))
                                .addTemporalMarker(3.0, () -> {
                                    // close claw
                                })
                                .addTemporalMarker(3.1, () -> {
                                    // straighten wrist
                                })
                                .addTemporalMarker(3.2, () -> {
                                    // swing arm
                                    // rotate cone
                                })
                                .addTemporalMarker(4.1, () -> {
                                    // wrist angle for cone pickup
                                })
                                .addTemporalMarker(4.2, () -> {
                                    // open claw
                                })

                                // slow down this portion of the trajectory
                                .splineToLinearHeading(new Pose2d(-58, -12, Math.toRadians(-180)), Math.toRadians(-180))
                                .addTemporalMarker(4.7, () -> {
                                    // lower slide elevator
                                })
                                .setReversed(true)
                                //2nd cone
                                .splineToSplineHeading(new Pose2d(-48, -12, Math.toRadians(-180)), Math.toRadians(0))
                                .splineToSplineHeading(junctionPos, Math.toRadians(45))
                                .waitSeconds(.75)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(-48, -12, Math.toRadians(-180)), Math.toRadians(-180))
                                .splineToLinearHeading(new Pose2d(-58, -12, Math.toRadians(-180)), Math.toRadians(-180))
                                .waitSeconds(.75)
                                .setReversed(true)
                                //3rd cone
                                .splineToSplineHeading(new Pose2d(-48, -12, Math.toRadians(-180)), Math.toRadians(0))
                                .splineToSplineHeading(junctionPos, Math.toRadians(45))
                                .waitSeconds(.75)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(-48, -12, Math.toRadians(-180)), Math.toRadians(-180))
                                .splineToLinearHeading(new Pose2d(-58, -12, Math.toRadians(-180)), Math.toRadians(-180))
                                .waitSeconds(.75)
                                .setReversed(true)
                                //4th cone
                                .splineToSplineHeading(new Pose2d(-48, -12, Math.toRadians(-180)), Math.toRadians(0))
                                .splineToSplineHeading(junctionPos, Math.toRadians(45))
                                .waitSeconds(.75)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(-48, -12, Math.toRadians(-180)), Math.toRadians(-180))
                                .splineToLinearHeading(new Pose2d(-58, -12, Math.toRadians(-180)), Math.toRadians(-180))
                                .waitSeconds(.75)
                                .setReversed(true)
                                //5th cone
                                .splineToSplineHeading(new Pose2d(-48, -12, Math.toRadians(-180)), Math.toRadians(0))
                                .splineToSplineHeading(junctionPos, Math.toRadians(45))
                                .waitSeconds(.75)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(-48, -12, Math.toRadians(-180)), Math.toRadians(-180))
                                .splineToLinearHeading(new Pose2d(-58, -12, Math.toRadians(-180)), Math.toRadians(-180))
                                .waitSeconds(.75)
                                .setReversed(true)
                                //6th cone
                                .splineToSplineHeading(new Pose2d(-48, -12, Math.toRadians(-180)), Math.toRadians(0))
                                .splineToSplineHeading(junctionPos, Math.toRadians(45))
                                .waitSeconds(.75)
                                .setReversed(false)
                                .splineToLinearHeading(new Pose2d(-58, -12, Math.toRadians(-180)), Math.toRadians(-180))
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