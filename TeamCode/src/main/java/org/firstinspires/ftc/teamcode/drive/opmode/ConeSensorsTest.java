package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
//@Disabled
//@TeleOp(group = "drive")
@TeleOp(name = "2 Distance sense test")
public class ConeSensorsTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose=new Pose2d(36,-64.5,Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DistanceSensor leftDistance;
        leftDistance = hardwareMap.get(DistanceSensor.class, "frontDistance");
        DistanceSensor rightDistance;
        rightDistance = hardwareMap.get(DistanceSensor.class, "rearDistance");
        double rawRangeRight;
        double rawRangeLeft;
        double angle1;
        double angle2;
        double distanceCenter;

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();
            rawRangeLeft = leftDistance.getDistance(DistanceUnit.INCH);
            rawRangeRight = rightDistance.getDistance(DistanceUnit.INCH);
            double rangeLeft = rawRangeLeft * 1.708300399 - 0.407779539;
            double rangeRight = rawRangeRight * 1.247496845 - 0.204661338;

//            angle1 = Math.atan((rangeLeft-rangeRight)/1.5);
//            angle2 = Math.acos((Math.sqrt(Math.pow((rangeLeft-rangeRight),2)+Math.pow(1.5,2)))/(2*1.55));
//            distanceCenter = 1.55*Math.acos(angle1+angle2);

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addData("left sensor distance: ", rangeLeft);
            telemetry.addData("right sensor distance: ", rangeRight);
            telemetry.addData("RAW left sensor distance: ", rawRangeLeft);
            telemetry.addData("RAW right sensor distance: ", rawRangeRight);
            telemetry.addData("Left(+)/Right(-): ", rangeRight - rangeLeft);
            telemetry.addData("Forward: ", ((rangeRight + rangeLeft)/2)-1);
            telemetry.update();
        }
    }
}
