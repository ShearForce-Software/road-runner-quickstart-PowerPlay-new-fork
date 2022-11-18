package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Disabled
@TeleOp(name = "Auto Drop")
public class AutoDrop extends LinearOpMode {

    DistanceSensor rearDistance;
    DistanceSensor clawDistance;
    DistanceSensor frontDistance;


    @Override
    public void runOpMode() throws InterruptedException {

        rearDistance = hardwareMap.get(DistanceSensor.class, "rearDistance");
        clawDistance = hardwareMap.get(DistanceSensor.class, "clawDistance");
        frontDistance = hardwareMap.get(DistanceSensor.class, "frontDistance");

        telemetry.addData(">", "Press Start and move the sensor to activate the servos." );
        telemetry.update();
        // wait for the start button to be pressed.

        waitForStart();

        while (opModeIsActive()) {

            double rearRange = rearDistance.getDistance(DistanceUnit.CM);
            double clawRange = clawDistance.getDistance(DistanceUnit.CM);
            double frontRange = frontDistance.getDistance(DistanceUnit.CM);

            telemetry.addData("rearDistance (cm): ","%.02f", rearRange);
            telemetry.addData("clawDistance (cm): ","%.02f", clawRange);
            telemetry.addData("frontDistance (cm): ","%.02f", frontRange);

            telemetry.update();
        }
    }
}


