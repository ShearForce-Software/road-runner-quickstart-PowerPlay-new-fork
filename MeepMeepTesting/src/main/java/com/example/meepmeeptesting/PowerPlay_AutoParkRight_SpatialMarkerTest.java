package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class PowerPlay_AutoParkRight_SpatialMarkerTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12.0)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(31, -3, Math.toRadians(-45)))
                                //~~~~parking~~~~
                                //*comment out the "/* */" of the park route you want to run
                                 /*
                                //park1
                                .splineToLinearHeading(new Pose2d(36, -24, Math.toRadians(-90)), Math.toRadians(-90))
                                .forward(12)
                                .strafeRight(24)
                                .addSpatialMarker(new Vector2d(36,-12),()->{
                                    //close claw
                                })
                                .addSpatialMarker(new Vector2d(36,-14),()->{
                                    //straighten wrist
                                })
                                .addSpatialMarker(new Vector2d(36,-16),()->{
                                    // swing arm
                                    // rotate cone
                                })
                                .addSpatialMarker(new Vector2d(36,-36),()->{
                                    // wrist angle
                                })
                                .addSpatialMarker(new Vector2d(34,-36),()->{
                                    // open claw
                                })
                                 */
                                //park 2
                                 /*
                                .splineToSplineHeading(new Pose2d(36, -24, Math.toRadians(-90)), Math.toRadians(-90))
                                .forward(12)
                                .addSpatialMarker(new Vector2d(36,-12),()->{
                                    //close claw
                                })
                                .addSpatialMarker(new Vector2d(36,-14),()->{
                                    //straighten wrist
                                })
                                .addSpatialMarker(new Vector2d(36,-16),()->{
                                    // swing arm
                                    // rotate cone
                                })
                                .addSpatialMarker(new Vector2d(36,-33),()->{
                                    // wrist angle
                                })
                                .addSpatialMarker(new Vector2d(36,-35),()->{
                                    // open claw
                                })
                                 */
                                // /*
                                //park 3
                                .splineToLinearHeading(new Pose2d(36, -24, Math.toRadians(-90)), Math.toRadians(-90))
                                .forward(12)
                                .strafeLeft(24)
                                .addSpatialMarker(new Vector2d(36,-12),()->{
                                    //close claw
                                })
                                .addSpatialMarker(new Vector2d(36,-14),()->{
                                    //straighten wrist
                                })
                                .addSpatialMarker(new Vector2d(36,-16),()->{
                                    // swing arm
                                    // rotate cone
                                })
                                .addSpatialMarker(new Vector2d(36,-36),()->{
                                    // wrist angle
                                })
                                .addSpatialMarker(new Vector2d(38,-36),()->{
                                    // open claw
                                })
                                // */
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}