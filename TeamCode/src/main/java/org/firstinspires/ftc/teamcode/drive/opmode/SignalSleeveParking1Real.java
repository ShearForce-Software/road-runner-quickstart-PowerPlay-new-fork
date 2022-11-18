package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Disabled
public class SignalSleeveParking1Real{
    public void DoParkLeft(HardwareMap hardwareMap) throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        String label = "Checkered1";

        Trajectory trajectory;
        if (label.equals("Checkered1")) {
            trajectory = drive.trajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(-12, 12, Math.toRadians(0)))
                    .build();
        }
        else if (label.equals("Logo3")){
            trajectory = drive.trajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(-36, 12, Math.toRadians(0)))
                    .build();
        }else{
            trajectory = drive.trajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(-60, 12, Math.toRadians(0)))
                    .build();
        }

        drive.followTrajectory(trajectory);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();
    }
}
