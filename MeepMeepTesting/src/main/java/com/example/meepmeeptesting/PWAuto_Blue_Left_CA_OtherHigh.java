package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class PWAuto_Blue_Left_CA_OtherHigh {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        Vector2d junctionVec = new Vector2d(-27,-7.4);
        Vector2d junctionTwoVec = new Vector2d(-6,-18);
        Pose2d stackPos = new Pose2d(-63, -12, Math.toRadians(180));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(200), Math.toRadians(200), 12.5)
                .setDimensions(15.5,15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -63, Math.toRadians(-90)))
                                //first Cone
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(-36, -20), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(junctionVec.getX()-(6*(1/Math.sqrt(2))), junctionVec.getY()-(6*(1/Math.sqrt(2))), Math.toRadians(-135)), Math.toRadians(45))
                                .splineToConstantHeading(junctionVec, Math.toRadians(45))
                                .waitSeconds(0.5)
                                //return to stack
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(-38, -12, Math.toRadians(180)), Math.toRadians(180))
                                .splineToLinearHeading(stackPos, Math.toRadians(180))
                                .waitSeconds(1.75)

                                //2nd cone
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(-20, -12), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(junctionTwoVec.getX()-(5*(1/Math.sqrt(2))), junctionTwoVec.getY()+(5*(1/Math.sqrt(2))), Math.toRadians(135)), Math.toRadians(-45))
                                .splineToConstantHeading(junctionTwoVec, Math.toRadians(-45))
                                .waitSeconds(1.25)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(-16,-12,Math.toRadians(180)), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(-50,-12,Math.toRadians(180)), Math.toRadians(180))
                                .splineToLinearHeading(stackPos, Math.toRadians(180))
                                .waitSeconds(2.0)

                                //3rd cone
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(-20, -12), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(junctionTwoVec.getX()-(5*(1/Math.sqrt(2))), junctionTwoVec.getY()+(5*(1/Math.sqrt(2))), Math.toRadians(135)), Math.toRadians(-45))
                                .splineToConstantHeading(junctionTwoVec, Math.toRadians(-45))
                                .waitSeconds(1.25)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(-16,-12,Math.toRadians(180)), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(-50,-12,Math.toRadians(180)), Math.toRadians(180))
                                .splineToLinearHeading(stackPos, Math.toRadians(180))
                                .waitSeconds(2.0)
                                //4th cone
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(-20, -12), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(junctionTwoVec.getX()-(5*(1/Math.sqrt(2))), junctionTwoVec.getY()+(5*(1/Math.sqrt(2))), Math.toRadians(135)), Math.toRadians(-45))
                                .splineToConstantHeading(junctionTwoVec, Math.toRadians(-45))
                                .waitSeconds(1.25)
                                .setReversed(false)
                                //>>>>>>>>>>>park<<<<<<<<<<<<<
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}