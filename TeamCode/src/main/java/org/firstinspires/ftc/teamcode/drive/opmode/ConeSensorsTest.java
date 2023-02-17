package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Locale;

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
            double rangeRedLeft, rangeRedRight, rangeBlueLeft, rangeBlueRight;
            //red
            if(rawRangeLeft < 1.78){
                rangeRedLeft = rawRangeLeft * 1.352 - 0.089;
            }
            else if(rawRangeLeft > 1.78){
                rangeRedLeft = rawRangeLeft * 3.333 - 3.777;
            }
            else{
                rangeRedLeft=rawRangeLeft;
            }
            if(rawRangeRight < 2.23){
                rangeRedRight = rawRangeRight * 1.050 - 0.040;
            }
            else if(rawRangeRight > 2.23){
                rangeRedRight = rawRangeRight * 2.038 - 2.399;
            }
            else{
                rangeRedRight=rawRangeRight;
            }
            //~~~~~~~~~~~~~~~~~blue~~~~~~~~~~~~~~~~```
            if(rawRangeLeft<2.0){
                rangeBlueLeft = rawRangeLeft * 1.290 - 0.292;
            }
            else if(rawRangeLeft>2.0){
                rangeBlueLeft = rawRangeLeft * 3.947 - 5.623;
            }
            else{
                rangeBlueLeft=rawRangeLeft;
            }
            if(rawRangeRight<2.55){
                rangeBlueRight = rawRangeRight * 0.948 - 0.085;
            }
            else if(rawRangeRight>2.55){
                rangeBlueRight = rawRangeRight * 2.838 - 5.360;
            }
            else{
                rangeBlueRight=rawRangeRight;
            }

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addData("sensor RED distance left:",String.format(Locale.US, "%.02f", rangeRedLeft));
            telemetry.addData("sensor RED distance Right:",String.format(Locale.US, "%.02f", rangeRedRight));
            telemetry.addData("sensor BLUE distance left:",String.format(Locale.US, "%.02f", rangeBlueLeft));
            telemetry.addData("sensor BLUE distance Right:",String.format(Locale.US, "%.02f", rangeBlueRight));
            telemetry.addData("Left(+)/Right(-) RED: ",String.format(Locale.US, "%.02f", rangeRedRight - rangeRedLeft));
            telemetry.addData("Left(+)/Right(-) BLUE: ",String.format(Locale.US, "%.02f", rangeBlueRight - rangeBlueLeft));
            //telemetry.addData("Forward: ", ((rangeRedRight + rangeRedLeft)/2)-1);
            telemetry.update();
        }
    }
}
