package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@TeleOp(name = "Auto Route Left Far")
public class Auto_Route_Left_Far extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        //sleep(2000);

        drive.followTrajectorySequence(
                drive.trajectorySequenceBuilder(new Pose2d(36, 36, Math.toRadians(-90)))
                        .splineToLinearHeading(new Pose2d(50, 12, Math.toRadians(-180)), Math.toRadians(0))
                        //1st cone
                        .splineToLinearHeading(new Pose2d(31, 7, Math.toRadians(135)), Math.toRadians(135))
                        .waitSeconds(.75)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(58, 12, Math.toRadians(-180)), Math.toRadians(0))
                        .waitSeconds(.75)
                        .setReversed(false)
                        //2nd cone
                        .splineToLinearHeading(new Pose2d(31, 7, Math.toRadians(135)), Math.toRadians(135))
                        .waitSeconds(.75)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(58, 12, Math.toRadians(-180)), Math.toRadians(0))
                        .waitSeconds(.75)
                        .setReversed(false)
                        //3rd cone
                        .splineToLinearHeading(new Pose2d(31, 7, Math.toRadians(135)), Math.toRadians(135))
                        .waitSeconds(.75)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(58, 12, Math.toRadians(-180)), Math.toRadians(0))
                        .waitSeconds(.75)
                        .setReversed(false)
                        //4th cone
                        .splineToLinearHeading(new Pose2d(31, 7, Math.toRadians(135)), Math.toRadians(135))
                        .waitSeconds(.75)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(58, 12, Math.toRadians(-180)), Math.toRadians(0))
                        .waitSeconds(.75)
                        .setReversed(false)
                        //5th cone
                        .splineToLinearHeading(new Pose2d(31, 7, Math.toRadians(135)), Math.toRadians(135))
                        .waitSeconds(.75)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(58, 12, Math.toRadians(-180)), Math.toRadians(0))
                        .waitSeconds(.75)
                        .setReversed(false)
                        //6th cone
                        .splineToLinearHeading(new Pose2d(31, 7, Math.toRadians(135)), Math.toRadians(135))
                        .build()
        );
    }
}
