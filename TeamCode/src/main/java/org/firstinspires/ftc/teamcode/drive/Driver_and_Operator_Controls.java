package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "MAIN_DRIVER_CONTROL_PROGRAM")

public class Driver_and_Operator_Controls extends LinearOpMode {

    // Define class members
    Servo  spinOne;
    Servo  spinTwo;
    Servo  armRote;
    Servo  liftWrist;
    Servo  armGrip;
    DcMotor slideOne;
    DcMotor slideTwo;
    DcMotor leftFront;
    DcMotor leftRear;
    DcMotor rightFront;
    DcMotor rightRear;
    DistanceSensor rearDistance;
    DistanceSensor clawDistance;
    DistanceSensor frontDistance;

    double  position1 = 0.95;
    double  position2 = 0.95;
    double  position3 = 0.13;
    double  position4 = 0.6;
    double  position5 = 0.0;
    int desiredPos = 0;
    public static final double ARM_POWER    =  1 ;
    static final double INCREMENT = 0.01;

    @Override
    public void runOpMode() throws InterruptedException {

        hardwareMap();
        motorInit();

        waitForStart();
        if (isStopRequested()) return;

        while(opModeIsActive()){
            driveControls(leftFront, leftRear, rightFront, rightRear);

            if (gamepad1.left_bumper) {
                position5 = .18;
            }
            if (gamepad1.right_bumper) {
                position5 = 0;
            }
            double rangeRear = rearDistance.getDistance(DistanceUnit.CM);
            double rangeClaw = clawDistance.getDistance(DistanceUnit.CM);
            double rangeFront = frontDistance.getDistance(DistanceUnit.CM);

            //AutoGrab
            if ((((rangeClaw < 2.75) && (position5 == .18)) && (slideOne.getCurrentPosition() <= 100) && (slideTwo.getCurrentPosition() <= 100))) {
                position5 = 0;
                position1 = .80;
                position2 = .80;
                desiredPos = 1400;
                armGrip.setPosition(position5);
                for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(200); stop>System.nanoTime();) {
                    driveControls(leftFront, leftRear, rightFront, rightRear);
                }
                slideOne.setTargetPosition(desiredPos);
                slideTwo.setTargetPosition(desiredPos);
                slideOne.setPower(ARM_POWER);
                slideTwo.setPower(ARM_POWER);
                slideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while ((slideOne.isBusy()) && (slideTwo.isBusy())){
                    driveControls(leftFront, leftRear, rightFront, rightRear);
                }
                slideOne.setPower(0);
                slideTwo.setPower(0);
                spinOne.setPosition(position1);
                spinTwo.setPosition(position2);
            }
            //High
            if (gamepad1.y){
                position5 = 0;
                position1 = .11;
                position2 = .11;
                position3 = .83;
                position4 = .13;
                armGrip.setPosition(position5);
                slideOne.setTargetPosition(1738);
                slideTwo.setTargetPosition(1738);
                slideOne.setPower(ARM_POWER);
                slideTwo.setPower(ARM_POWER);
                slideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while ((slideOne.isBusy()) && (slideTwo.isBusy())){
                    driveControls(leftFront, leftRear, rightFront, rightRear);
                }
                slideOne.setPower(0);
                slideTwo.setPower(0);
                spinOne.setPosition(position1);
                spinTwo.setPosition(position2);
                for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(1000); stop>System.nanoTime();) {
                    driveControls(leftFront, leftRear, rightFront, rightRear);
                }
                armRote.setPosition(position3);
                liftWrist.setPosition(position4);
            }
            //Medium
            if (gamepad1.x){
                position5 = 0;
                position1 = .48;
                position2 = .48;
                position3 = .83;
                position4 = .86;
                armGrip.setPosition(position5);
                slideOne.setTargetPosition(3681);
                slideTwo.setTargetPosition(3681);
                slideOne.setPower(ARM_POWER);
                slideTwo.setPower(ARM_POWER);
                slideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while ((slideOne.isBusy()) && (slideTwo.isBusy())){
                    driveControls(leftFront, leftRear, rightFront, rightRear);
                }
                slideOne.setPower(0);
                slideTwo.setPower(0);
                spinOne.setPosition(position1);
                spinTwo.setPosition(position2);
                for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(300); stop>System.nanoTime();) {
                    driveControls(leftFront, leftRear, rightFront, rightRear);
                }
                armRote.setPosition(position3);
                liftWrist.setPosition(position4);
                for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(400); stop>System.nanoTime();) {
                    driveControls(leftFront, leftRear, rightFront, rightRear);
                }
                position1 = .83;
                position2 = .83;
                spinOne.setPosition(position1);
                spinTwo.setPosition(position2);
            }
            //Low
            if (gamepad1.a){
                position5 = 0;
                position1 = .48;
                position2 = .48;
                position3 = .83;
                position4 = .82;
                armGrip.setPosition(position5);
                slideOne.setTargetPosition(2090);
                slideTwo.setTargetPosition(2090);
                slideOne.setPower(ARM_POWER);
                slideTwo.setPower(ARM_POWER);
                slideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while ((slideOne.isBusy()) && (slideTwo.isBusy())){
                    driveControls(leftFront, leftRear, rightFront, rightRear);
                }
                slideOne.setPower(0);
                slideTwo.setPower(0);
                spinOne.setPosition(position1);
                spinTwo.setPosition(position2);
                for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(500); stop>System.nanoTime();) {
                    driveControls(leftFront, leftRear, rightFront, rightRear);
                }
                armRote.setPosition(position3);
                liftWrist.setPosition(position4);
                for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(400); stop>System.nanoTime();) {
                    driveControls(leftFront, leftRear, rightFront, rightRear);
                }
                position1 = .83;
                position2 = .83;
                spinOne.setPosition(position1);
                spinTwo.setPosition(position2);
            }
            //ground pos
            if (gamepad1.b){
                position5 = 0;
                position1 = .48;
                position2 = .48;
                position3 = .13;
                position4 = .74;
                position5 = 0;
                desiredPos = 5;
                armGrip.setPosition(position5);
                spinOne.setPosition(position1);
                spinTwo.setPosition(position2);
                for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(500); stop>System.nanoTime();) {
                    driveControls(leftFront, leftRear, rightFront, rightRear);
                }
                armGrip.setPosition(position5);
                liftWrist.setPosition(position4);
                armRote.setPosition(position3);
                position1 = .95;
                position2 = .95;
                for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(700); stop>System.nanoTime();) {
                    driveControls(leftFront, leftRear, rightFront, rightRear);
                }
                spinOne.setPosition(position1);
                spinTwo.setPosition(position2);
                slideOne.setTargetPosition(desiredPos);
                slideTwo.setTargetPosition(desiredPos);
                slideOne.setPower(ARM_POWER);
                slideTwo.setPower(ARM_POWER);
                slideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while ((slideOne.isBusy()) && (slideTwo.isBusy())){
                    driveControls(leftFront, leftRear, rightFront, rightRear);
                }
                slideOne.setPower(0);
                slideTwo.setPower(0);
                position4 = .6;
                liftWrist.setPosition(position4);
                position5 = .18;
                armGrip.setPosition(position5);
            }

            // Display the current value
            telemetry.addData("spinOne Servo Position", "%5.2f", position1);
            telemetry.addData("spinTwo Servo Position", "%5.2f", position2);
            telemetry.addData("armRote Servo Position", "%5.2f", position3);
            telemetry.addData("liftWrist Servo Position", "%5.2f", position4);
            telemetry.addData("armGrip Servo Position", "%5.2f", position5);
            telemetry.addData("slideOne motor Position", slideOne.getCurrentPosition());
            telemetry.addData("slideTwo motor Position", slideTwo.getCurrentPosition());
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the servos to the new positions
            spinOne.setPosition(position1);
            spinTwo.setPosition(position2);
            armRote.setPosition(position3);
            liftWrist.setPosition(position4);
            armGrip.setPosition(position5);
            idle();
        }
    }

    private void motorInit() {
        // Directions
        spinOne.setDirection(Servo.Direction.FORWARD);
        spinTwo.setDirection(Servo.Direction.REVERSE);
        armRote.setDirection(Servo.Direction.FORWARD);
        liftWrist.setDirection(Servo.Direction.FORWARD);
        armGrip.setDirection(Servo.Direction.FORWARD);
        slideOne.setDirection(DcMotor.Direction.FORWARD);
        slideTwo.setDirection(DcMotor.Direction.FORWARD);
        slideOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        slideOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Wait for the start button
        telemetry.addData(">", "Press Start to move Servos with gamepad1." );
        telemetry.update();
    }

    private void hardwareMap() {
        // Servo names in hardware map on Control or Expansion Hubs
        rearDistance = hardwareMap.get(DistanceSensor.class, "rearDistance");
        clawDistance = hardwareMap.get(DistanceSensor.class, "clawDistance");
        frontDistance = hardwareMap.get(DistanceSensor.class, "frontDistance");

        spinOne = hardwareMap.get(Servo.class, "spinOne");
        spinTwo = hardwareMap.get(Servo.class, "spinTwo");
        armRote = hardwareMap.get(Servo.class, "armRote");
        liftWrist = hardwareMap.get(Servo.class, "liftWrist");
        armGrip = hardwareMap.get(Servo.class, "armGrip");

        slideOne = hardwareMap.get(DcMotor.class, "slideOne");
        slideTwo = hardwareMap.get(DcMotor.class, "slideTwo");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightRear = hardwareMap.dcMotor.get("rightRear");
    }

    private void driveControls(DcMotor leftFront, DcMotor leftRear, DcMotor rightFront, DcMotor rightRear) {
        double y = gamepad2.left_stick_y;
        double x = -gamepad2.left_stick_x * 1.1;
        double rx = gamepad2.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);
    }
}
