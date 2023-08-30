package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static double startX = 36;            // added start variable
    public static double startY = -64.5;          // added start variable
    public static double startHeading = -90;      // added start variable
    public static double stackY = -14;
    public static double stackX = 61;
    public static double junctionX = 30;
    public static double junctionY = -6;
    public static double junction1X = 30;          // added for to ID specific junction X
    public static double junction1Y = -6;           // added for to ID specific junction Y
    public static double junction1Heading = -45;     // added for to ID specific junction Heading
    public static double junction2X = 30;            // added for to ID specific junction X
    public static double junction2Y = -6;             // added for to ID specific junction Y
    public static double junction2Heading = -45;     // added for to ID specific junction Heading
    public static double numConesStack = 4;
    public static double toFirstConeVel = 55;
    public static double toStackVel = 35;
    public static double toHighVel = 35;
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        Pose2d startPose = new Pose2d(startX, startY, Math.toRadians(startHeading));  // added to replace line above
        Vector2d junction1Vec = new Vector2d(junction1X, junction1Y);
        Pose2d junction1Pos = new Pose2d(junction1X,junction1Y, Math.toRadians(junction1Heading));
        Pose2d stackPos = new Pose2d(stackX, stackY, Math.toRadians(0));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(68, 68, Math.toRadians(180), Math.toRadians(180), 12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(36,-64.5, Math.toRadians(-90)))
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(36, -20), Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(junction1Vec.getX()+(2*(1/Math.sqrt(2))), junction1Vec.getY()-(2*(1/Math.sqrt(2))), Math.toRadians(-45)), Math.toRadians(135))
                                .splineToConstantHeading(junction1Vec, Math.toRadians(135))
                                .setReversed(false)
                                .splineToLinearHeading(new Pose2d(38, stackY, Math.toRadians(0)), Math.toRadians(0))
                                .splineToLinearHeading(stackPos, Math.toRadians(0))
                                .setReversed(true)
                                .strafeTo(new Vector2d(44,stackY))
                                .splineToLinearHeading(new Pose2d(36, stackY, Math.toRadians(-45)), Math.toRadians(180))
                                .splineToConstantHeading(junction1Vec, Math.toRadians(135))
                                .splineToLinearHeading(new Pose2d(34.5, -14, Math.toRadians(-90)), Math.toRadians(-90))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}