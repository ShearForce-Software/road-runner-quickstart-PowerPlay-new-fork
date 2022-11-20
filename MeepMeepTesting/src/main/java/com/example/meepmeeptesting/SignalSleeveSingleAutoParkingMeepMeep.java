package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class SignalSleeveSingleAutoParkingMeepMeep {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = ParkRobot(meepMeep);

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    public static RoadRunnerBotEntity ParkRobot(MeepMeep meepMeep) {
        String label = "check";
        RoadRunnerBotEntity myBot;
        //This is all set up for far right
        if (label.equals("check")) {
            myBot = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(-8, 32, Math.toRadians(-45))) //set up for far right, use -135 for left
                                    .lineToLinearHeading(new Pose2d(-12, 36, Math.toRadians(90))) //far right
                                    //.lineToLinearHeading(new Pose2d(12, 36, Math.toRadians(90))) //far left
                                    .build()
                    );
        } else if (label.equals("logo")){
            myBot = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(-8, 32, Math.toRadians(-45))) //set up for far right, use -135 for left
                                    //far right
                                    .setReversed(true)
                                    .splineToSplineHeading(new Pose2d(-24, 36, Math.toRadians(0)), Math.toRadians(180))
                                    .lineToLinearHeading(new Pose2d(-60, 36, Math.toRadians(0)))
                                    //far left

                                    .build()
                    );
        } else {
            myBot = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(-8, 32, -45)) //set up for far right, use -135 for left
                                    //far right
                                    .setReversed(true)
                                    .lineToLinearHeading(new Pose2d(-36, 36, Math.toRadians(0)))
                                    //far left
                                    .build()
                    );
        }
        return myBot;
    }
}