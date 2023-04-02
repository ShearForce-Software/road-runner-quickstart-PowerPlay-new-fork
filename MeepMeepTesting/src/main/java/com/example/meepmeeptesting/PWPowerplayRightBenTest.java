package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class PWPowerplayRightBenTest {
    public static void main(String[] args) {
        //~~~~~~~~~~THIS CONTAINS ALL PARKING FOR BOTH SIDES~~~~~~~~~~~~~~
        int parkSpot = 3;
        boolean left = true;
        MeepMeep meepMeep = new MeepMeep(600);
        Pose2d junctionPos;

        if(left){//~~~~~~~~~~~~~~LEFT SIDE~~~~~~~~~~~~
            junctionPos = new Pose2d(-26,-6, Math.toRadians(-135));
            double x = (Math.sqrt(288)-Math.abs(junctionPos.getY())*Math.sqrt(2)-2.5);
            if (parkSpot == 3) {//Spot 3
                RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12.0)
                        .setDimensions(16, 16)
                        .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(junctionPos)
                                        .forward(2.5)
                                        .splineToSplineHeading(new Pose2d(junctionPos.getX()-(6*(1/Math.sqrt(2))), junctionPos.getY()-(6*(1/Math.sqrt(2))), Math.toRadians(180)), Math.toRadians(-135))
                                        // ^some really exact math :)
                                        .splineToConstantHeading(new Vector2d(-24, -12), Math.toRadians(0))
                                        .splineToConstantHeading(new Vector2d(-12, -12), Math.toRadians(0))
                                        .build()
                        );

                meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                        .setDarkMode(true)
                        .setBackgroundAlpha(0.95f)
                        .addEntity(myBot)
                        .start();
            } else if (parkSpot == 2) {//Spot 2
                RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12.0)
                        .setDimensions(16, 16)
                        .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(junctionPos)
                                        .splineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(180)), Math.toRadians(-135))
                                        .build()
                        );

                meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                        .setDarkMode(true)
                        .setBackgroundAlpha(0.95f)
                        .addEntity(myBot)
                        .start();
            } else {//Spot 1
                RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12.0)
                        .setDimensions(16, 16)
                        .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(junctionPos)
                                        .splineToSplineHeading(new Pose2d(-48, -12, Math.toRadians(180)), Math.toRadians(180))
                                        .forward(10)
                                        .build()
                        );

                meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                        .setDarkMode(true)
                        .setBackgroundAlpha(0.95f)
                        .addEntity(myBot)
                        .start();
            }
        }
        else {//~~~~~~~~~~RIGHT SIDE~~~~~~~~~~~~~
            junctionPos = new Pose2d(29, -6.4, Math.toRadians(-45));
            if (parkSpot == 1) {//Spot 1
                RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12.0)
                        .setDimensions(16, 16)
                        .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(junctionPos)
                                        .forward(2.5)
                                        .splineToSplineHeading(new Pose2d(29 + ((5.0 * Math.sqrt(2)) / 2.0), -6.4 - ((5.0 * Math.sqrt(2)) / 2.0), Math.toRadians(0)), Math.toRadians(-45))
                                        // ^some really exact math :)
                                        .splineToConstantHeading(new Vector2d(24, -12), Math.toRadians(180))
                                        .splineToSplineHeading(new Pose2d(12, -12, Math.toRadians(0)), Math.toRadians(180))
                                        .build()
                        );

                meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                        .setDarkMode(true)
                        .setBackgroundAlpha(0.95f)
                        .addEntity(myBot)
                        .start();
            } else if (parkSpot == 2) {//Spot 2
                RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12.0)
                        .setDimensions(16, 16)
                        .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(junctionPos)
                                        .splineToLinearHeading(new Pose2d(36, -12, Math.toRadians(0)), Math.toRadians(-45))
                                        .build()
                        );

                meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                        .setDarkMode(true)
                        .setBackgroundAlpha(0.95f)
                        .addEntity(myBot)
                        .start();
            } else {//Spot 3
                RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12.0)
                        .setDimensions(16, 16)
                        .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(junctionPos)
                                        .splineToSplineHeading(new Pose2d(48, -12, Math.toRadians(0)), Math.toRadians(0))
                                        .forward(10)
                                        .build()
                        );

                meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                        .setDarkMode(true)
                        .setBackgroundAlpha(0.95f)
                        .addEntity(myBot)
                        .start();
            }
        }
    }
}