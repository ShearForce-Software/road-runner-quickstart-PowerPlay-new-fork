package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "REVDistance")
public class SensorREVColorDistance_Demo extends LinearOpMode {

    DistanceSensor sensorDistance;

    @Override
    public void runOpMode() {

        sensorDistance = hardwareMap.get(DistanceSensor.class, "distance_sensor");

        // wait for the start button to be pressed.

        waitForStart();

        while (opModeIsActive()) {

            double range = sensorDistance.getDistance(DistanceUnit.INCH);

            telemetry.addData("Distance (inch): ","%.02f", range);
            telemetry.update();
        }
    }
}


