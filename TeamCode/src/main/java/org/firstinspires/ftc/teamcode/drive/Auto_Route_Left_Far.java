package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@TeleOp(name = "Auto Route")
public class Auto_Route_Left_Far extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        //sleep(2000);

        drive.followTrajectorySequence(
                drive.trajectorySequenceBuilder(new Pose2d(-36, 36, Math.toRadians(-90)))
                        .splineToLinearHeading(new Pose2d(-50, 12, Math.toRadians(0)), Math.toRadians(-180))
                        //1st cone
                        .splineToLinearHeading(new Pose2d(-31, 7, Math.toRadians(-45)), Math.toRadians(-45))
                        .waitSeconds(.75)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-58, 12, Math.toRadians(0)), Math.toRadians(-180))
                        .waitSeconds(.75)
                        .setReversed(false)
                        //2nd cone
                        .splineToLinearHeading(new Pose2d(-31, 7, Math.toRadians(-45)), Math.toRadians(-45))
                        .waitSeconds(.75)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-58, 12, Math.toRadians(0)), Math.toRadians(-180))
                        .waitSeconds(.75)
                        .setReversed(false)
                        //3rd cone
                        .splineToLinearHeading(new Pose2d(-31, 7, Math.toRadians(-45)), Math.toRadians(-45))
                        .waitSeconds(.75)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-58, 12, Math.toRadians(0)), Math.toRadians(-180))
                        .waitSeconds(.75)
                        .setReversed(false)
                        //4th cone
                        .splineToLinearHeading(new Pose2d(-31, 7, Math.toRadians(-45)), Math.toRadians(-45))
                        .waitSeconds(.75)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-58, 12, Math.toRadians(0)), Math.toRadians(-180))
                        .waitSeconds(.75)
                        .setReversed(false)
                        //5th cone
                        .splineToLinearHeading(new Pose2d(-31, 7, Math.toRadians(-45)), Math.toRadians(-45))
                        .waitSeconds(.75)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-58, 12, Math.toRadians(0)), Math.toRadians(-180))
                        .waitSeconds(.75)
                        .setReversed(false)
                        //6th cone
                        .splineToLinearHeading(new Pose2d(-31, 7, Math.toRadians(-45)), Math.toRadians(-45))
                        .waitSeconds(.75)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-58, 12, Math.toRadians(0)), Math.toRadians(-180))
                        .forward(48)
                        .build()
                //drive.trajectoryBuilder(traj.end(), true)
                        //.splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        //.build()
        );
    }
}
