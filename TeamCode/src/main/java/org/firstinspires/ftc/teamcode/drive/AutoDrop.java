package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Auto Drop")
public class AutoDrop extends LinearOpMode {

    DistanceSensor sensorDistance;


    Servo  spinOne;
    Servo  spinTwo;
    Servo  armGrip;
    double  position1 = 0.0;
    double  position2 = 0.0;
    double  position3 = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        sensorDistance = hardwareMap.get(DistanceSensor.class, "distance_sensor");
        spinOne = hardwareMap.get(Servo.class, "spinOne");
        spinTwo = hardwareMap.get(Servo.class, "spinTwo");
        armGrip = hardwareMap.get(Servo.class, "armGrip");
        spinOne.setDirection(Servo.Direction.FORWARD);
        spinTwo.setDirection(Servo.Direction.REVERSE);
        armGrip.setDirection(Servo.Direction.FORWARD);
        telemetry.addData(">", "Press Start and move the sensor to activate the servos." );
        telemetry.update();
        // wait for the start button to be pressed.

        waitForStart();

        while (opModeIsActive()) {

            double range = sensorDistance.getDistance(DistanceUnit.CM);
            if (range < 2) {
                position1 = 1;
                position2 = 1;
                position3 = 1;

                spinOne.setPosition(position1);
                spinTwo.setPosition(position2);
                sleep(3000);
                armGrip.setPosition(position3);
            }

            telemetry.addData("Distance (cm): ","%.02f", range);
            telemetry.update();
        }
    }
}


