package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Locale;

//@Disabled
@TeleOp(name = "2 Sensor Data Collect")

public class Cone_2_Sensor_Data_Collect extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //Pose2d startPose=new Pose2d(36,-64.5,Math.toRadians(90));
        //drive.setPoseEstimate(startPose);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DistanceSensor leftDistance;
        leftDistance = hardwareMap.get(DistanceSensor.class, "frontDistance");
        DistanceSensor rightDistance;
        rightDistance = hardwareMap.get(DistanceSensor.class, "rearDistance");
        double raw_rangeRight;
        double raw_rangeLeft;


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
            raw_rangeLeft = leftDistance.getDistance(DistanceUnit.INCH);
            raw_rangeRight = rightDistance.getDistance(DistanceUnit.INCH);
            double rangeRedLeft, rangeRedRight, rangeBlueLeft, rangeBlueRight;
            double shiftRed, shiftBlue, forwardRed, forwardBlue;

            // Two Calibration Equations for Red Cone
            if (raw_rangeLeft < 1.78) {
                rangeRedLeft = raw_rangeLeft * 1.352 - 0.089;
            }
            else if (raw_rangeLeft > 1.78) {
                rangeRedLeft = raw_rangeLeft * 3.333 - 3.777;
            }
            else {
                rangeRedLeft = raw_rangeLeft;
            }

            if (raw_rangeRight < 2.23) {
                rangeRedRight = raw_rangeRight * 1.050 - 0.040;
            }
            else if (raw_rangeLeft > 2.23) {
                rangeRedRight = raw_rangeRight * 2.038 - 2.399;
            }
            else {
                rangeRedRight = raw_rangeRight;
            }
            shiftRed = (rangeRedRight - rangeRedLeft) * .8;
            forwardRed = ((rangeRedRight + rangeRedLeft)/2) - .6;

            if (shiftRed > 1) {
                shiftRed = 1;
                forwardRed = rangeRedLeft - 0.25;
            }
            else if (shiftRed < -1){
                shiftRed = -1;
                forwardRed = rangeRedRight - 0.25;
            }

            // Two Calibration Equations for Red BlueCone
            if (raw_rangeLeft < 2.0) {
                rangeBlueLeft = raw_rangeLeft * 1.290 - 0.292;
            }
            else if (raw_rangeLeft > 2.0) {
                rangeBlueLeft = raw_rangeLeft * 3.947 - 5.623;
            }
            else {
                rangeBlueLeft = raw_rangeLeft;
            }

            if (raw_rangeRight < 2.55) {
                rangeBlueRight = raw_rangeRight * 0.948 - 0.085;
            }
            else if (raw_rangeLeft > 2.55) {
                rangeBlueRight = raw_rangeRight * 2.838 - 5.360;
            }
            else{
                rangeBlueRight = raw_rangeRight;
            }
            shiftBlue = (rangeBlueRight - rangeBlueLeft) * .85;
            forwardBlue = ((rangeBlueRight + rangeBlueLeft)/2) - .5;

            if (shiftBlue > 1) {
                shiftBlue = 1;
                forwardBlue = rangeBlueLeft - 0.35;
            }
            else if (shiftBlue < -1){
                shiftBlue = -1;
                forwardBlue = rangeBlueRight - 0.35;
            }

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", "%6.2f in",poseEstimate.getX());
            telemetry.addData("y", "%6.2f in", poseEstimate.getY());
            telemetry.addData("heading", "%6.2f deg", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addData("RED distance left/right","%.2f in   %.2f in", rangeRedLeft, rangeRedRight);
            telemetry.addData("BLUE distance left/right","%.2f in   %.2f in", rangeBlueLeft, rangeBlueRight);
            //telemetry.addData("Left(+)/Right(-) RED: ", "%.2f", shiftRed);
            //telemetry.addData("Left(+)/Right(-) BLUE: ","%.2f", shiftBlue);
            //telemetry.addData("Forward RED ","%.2f", forwardRed);
            //telemetry.addData("Forward BLUE ", "%.2f", forwardBlue);
            telemetry.update();
        }
    }
}
