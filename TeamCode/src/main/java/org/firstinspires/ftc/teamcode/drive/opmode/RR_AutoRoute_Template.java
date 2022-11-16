package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * Op mode to experiment with Road Runner Autonomous Routes.
 * Utilization of the dashboard is recommended to visualize routes and check path following.
 * To access the dashboard, connect your computer to the RC's WiFi network.
 * In your browser, navigate to https://192.168.49.1:8080/dash if you're using the RC phone
 * or https://192.168.43.1:8080/dash if you are using the Control Hub.
 * Once successfully connected, start the program, and Li'l Gerry will begin driving the route.
 * You should observe the target position (green) and your pose estimate (blue) and adjust your
 * follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 */
@Config
@Autonomous(group = "drive")
public class RR_AutoRoute_Template extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Starting position of robot on field
        Pose2d startPose = new Pose2d(16, 60, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        TrajectorySequence lil_gerry = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(3)
                .splineToConstantHeading(new Vector2d(-12, 40), Math.toRadians(-90))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(16, 60, Math.toRadians(0)), Math.toRadians(0))
                .forward(36)
                .back(36)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-12, 40, Math.toRadians(-90)), Math.toRadians(-90))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(16, 60, Math.toRadians(0)), Math.toRadians(0))
                .forward(36)
                .back(36)
                .build();
        drive.followTrajectorySequence(lil_gerry);

    }
}
