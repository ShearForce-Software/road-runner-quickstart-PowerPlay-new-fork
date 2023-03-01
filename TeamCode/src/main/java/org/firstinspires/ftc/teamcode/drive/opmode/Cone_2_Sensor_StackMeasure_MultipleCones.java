package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Disabled
@TeleOp(name = "2 Cone Auto Detect and Measure - Multiple")
//@Disabled                            // Comment this out to add to the opmode list
public class Cone_2_Sensor_StackMeasure_MultipleCones extends LinearOpMode {


    ColorSensor sensorColorLeft;
    ColorSensor sensorColorRight;
    DistanceSensor sensorDistanceLeft;
    DistanceSensor sensorDistanceRight;
    double rawRangeRight;
    double rawRangeLeft;
    double rangeLeft = 0.0;
    double rangeRight = 0.0;
    double forwardLG = 0.0;
    double shiftLG = 0.0;
    double shiftRedScaleValue = 0.85;
    double shiftBlueScaleValue = 0.85;
    double forwardRed2SensorScaleValue = 0.1;
    double forwardBlue2SensorScaleValue = 0.15;
    double forwardRed1SensorScaleValue = -0.2;
    double forwardBlue1SensorScaleValue = -0.1;
    int redConeDetect = 0;
    int blueConeDetect = 0;



    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose=new Pose2d(0,0,Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        // get a reference to the color sensor.
        sensorColorLeft = hardwareMap.get(ColorSensor.class, "frontDistance");
        sensorColorRight = hardwareMap.get(ColorSensor.class, "rearDistance");

        // get a reference to the distance sensor that shares the same name.
        sensorDistanceLeft = hardwareMap.get(DistanceSensor.class, "frontDistance");
        sensorDistanceRight = hardwareMap.get(DistanceSensor.class, "rearDistance");

        // wait for the start button to be pressed.
        waitForStart();

        while (opModeIsActive()) {

            rawRangeLeft = sensorDistanceLeft.getDistance(DistanceUnit.INCH);
            rawRangeRight = sensorDistanceRight.getDistance(DistanceUnit.INCH);

            redConeDetect = (sensorColorLeft.red() + sensorColorRight.red()) / 2;
            blueConeDetect = (sensorColorLeft.blue() + sensorColorRight.blue()) / 2;

            if (redConeDetect > blueConeDetect) {
                if (rawRangeLeft < 1.78) {
                    rangeLeft = (rawRangeLeft * 1.352 - 0.089) - 0.15;
                } else if (rawRangeLeft > 1.78) {
                    rangeLeft = (rawRangeLeft * 3.333 - 3.777) - 0.15;
                } else {
                    rangeLeft = rawRangeLeft;
                }
                if (rawRangeRight < 2.23) {
                    rangeRight = (rawRangeRight * 1.050 - 0.040) - 0.15;
                } else if (rawRangeRight > 2.23) {
                    rangeRight = (rawRangeRight * 2.038 - 2.399) - 0.15;
                } else {
                    rangeRight = rawRangeRight;
                }

                shiftLG = (rangeRight - rangeLeft) * shiftRedScaleValue;
                forwardLG = ((rangeRight + rangeLeft)/2) - forwardRed2SensorScaleValue;

                if (shiftLG > 1.1) {
                    shiftLG = 1.25;
                }
                else if (shiftLG < -1.1){
                    shiftLG = -1.25;
                }
                if (shiftLG > 0.5) {
                    forwardLG = rangeLeft - forwardRed1SensorScaleValue;
                }
                else if (shiftLG < -0.5){
                    forwardLG = rangeRight - forwardRed1SensorScaleValue;
                }
            }

            //if ((sensorColorLeft.blue() > 75 || sensorColorRight.blue() > 75) && (sensorColorLeft.blue() > sensorColorLeft.red())) {
            if (blueConeDetect > redConeDetect) {
                if (rawRangeLeft < 2.0) {
                    rangeLeft = (rawRangeLeft * 1.290 - 0.292) - 0.1;
                } else if (rawRangeLeft > 2.0) {
                    rangeLeft = (rawRangeLeft * 3.947 - 5.623) - 0.1;
                } else {
                    rangeLeft = rawRangeLeft;
                }
                if (rawRangeRight < 2.55) {
                    rangeRight = (rawRangeRight * 0.948 - 0.085) - 0.1;
                } else if (rawRangeRight > 2.55) {
                    rangeRight = (rawRangeRight * 2.838 - 5.360) - 0.1;
                } else {
                    rangeRight = rawRangeRight;
                }
                shiftLG = (rangeRight - rangeLeft) * shiftBlueScaleValue;
                forwardLG = ((rangeRight + rangeLeft)/2) - forwardBlue2SensorScaleValue;

                if (shiftLG > 1.1) {
                    shiftLG = 1.25;
                }
                else if (shiftLG < -1.1){
                    shiftLG = -1.25;
                }
                if (shiftLG > 0.5) {
                    forwardLG = rangeLeft - forwardBlue1SensorScaleValue;
                }
                else if (shiftLG < -0.5){
                    forwardLG = rangeRight - forwardBlue1SensorScaleValue;
                }

            }


            // send the info back to driver station using telemetry function.
            telemetry.addData("Raw Distance Left (in)","%.2f", sensorDistanceLeft.getDistance(DistanceUnit.INCH));
            telemetry.addData("Raw Distance Right (in)","%.2f", sensorDistanceRight.getDistance(DistanceUnit.INCH));

            telemetry.addData("Auto Distance Left / Right (in)","%.2f in   %.2f in", rangeLeft, rangeRight);
            telemetry.addData("Shift Distance Left / Right (in)","%.2f in", shiftLG);
            telemetry.addData("Forward Distance (in)","%.2f in", forwardLG);


            telemetry.addData("Left / Right Red ", "%3d  %3d", sensorColorLeft.red(),sensorColorRight.red());
            telemetry.addData("Left / Right Blue ", "%3d  %3d", sensorColorLeft.blue(),sensorColorRight.blue());

            telemetry.addData("Red Detect / Blue Detect ", "%3d  %3d", redConeDetect, blueConeDetect);

            telemetry.update();
        }


    }
}
