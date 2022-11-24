package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class SignalSleeveFullAutoParkingMeepMeep {
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
        String label = "hj";
        RoadRunnerBotEntity myBot;

        if (label.equals("check")) {
            myBot = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(-8, 20, Math.toRadians(205)))
                                    .lineToLinearHeading(new Pose2d(-12, 12, Math.toRadians(180)))
                                    .build()
                    );
        } else if (label.equals("logo")){
            myBot = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(-8, 20, Math.toRadians(205)))
                                    .splineToLinearHeading(new Pose2d(-60, 12, Math.toRadians(180)), Math.toRadians(180))
                                    .build()
                    );
        } else {
            myBot = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(-8, 20, Math.toRadians(205)))
                                    .splineToLinearHeading(new Pose2d(-36, 12, Math.toRadians(180)), Math.toRadians(180))
                                    .build()
                    );
        }
        return myBot;
    }
}