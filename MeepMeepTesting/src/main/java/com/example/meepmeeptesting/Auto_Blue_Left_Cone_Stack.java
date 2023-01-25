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
        Pose2d startPose = new Pose2d(-36, -64.5, Math.toRadians(90));
        Vector2d junctionVec = new Vector2d(-26.3,-7.3);
        Pose2d junctionPos = new Pose2d(-26.3,-7.3, Math.toRadians(-135));
        Pose2d stackPos = new Pose2d(-62.5, -12, Math.toRadians(180));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12.0)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .strafeTo(new Vector2d(-36,-56))
                                .splineToSplineHeading(new Pose2d(-36, -28, Math.toRadians(180)), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(-36, -24), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(-36, -16.5, Math.toRadians(-135)), Math.toRadians(90))
                                .splineToConstantHeading(junctionVec, Math.toRadians(45))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}