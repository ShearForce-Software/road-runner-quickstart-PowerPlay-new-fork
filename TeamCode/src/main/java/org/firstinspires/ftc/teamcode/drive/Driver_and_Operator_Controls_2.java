package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.TimeUnit;


@TeleOp(name = "Stack Pickup Not Final")

public class Driver_and_Operator_Controls_2 extends LinearOpMode {

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

    double  position1 = 1.2;
    double  position2 = 1.2;
    double  position3 = 0.13;
    double  position4 = 0.6;
    double  position5 = 0.0;
    int desiredPos = 0;
    boolean high = false;
    boolean movingDown = false;
    double ARM_POWER    =  1 ;
    boolean stackPickUp = false;

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
            if (gamepad1.dpad_up){
                position5 = 0;
                desiredPos = 1600;
                slideOne.setTargetPosition(desiredPos);
                slideTwo.setTargetPosition(desiredPos);
                slideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideOne.setPower(ARM_POWER);
                slideTwo.setPower(ARM_POWER);
                for (long stop = System.nanoTime() + TimeUnit.MILLISECONDS.toNanos(1800); stop > System.nanoTime(); ) {}
                stackPickUp = true;
                while (stackPickUp){
                    desiredPos = slideOne.getCurrentPosition() - 10;
                    slideOne.setPower(ARM_POWER);
                    slideTwo.setPower(ARM_POWER);
                    slideOne.setTargetPosition(desiredPos);
                    slideTwo.setTargetPosition(desiredPos);
                    if (rangeClaw <= 3){
                        stackPickUp = false;
                        armGrip.setPosition(position5);
                        desiredPos = 2000;
                        for (long stop = System.nanoTime() + TimeUnit.MILLISECONDS.toNanos(500); stop > System.nanoTime(); ) {}
                        slideOne.setPower(ARM_POWER);
                        slideTwo.setPower(ARM_POWER);
                        slideOne.setTargetPosition(desiredPos);
                        slideTwo.setTargetPosition(desiredPos);
                        slideOne.setPower(0);
                        slideTwo.setPower(0);
                    if (slideOne.getCurrentPosition() <= 50){
                        break;
                    }

                    }
                }
            }
            /*
            //AutoGrab
            if ((((rangeClaw < 2.75) && (position5 == .18)) && (slideOne.getCurrentPosition() <= 100) && (slideTwo.getCurrentPosition() <= 100))) {
                position5 = 0;
                position1 = .72;
                position2 = .72;
                position4 = .35;
                position3 = .83;
                desiredPos = 1400;
                armGrip.setPosition(position5);
                for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(200); stop>System.nanoTime();) {
                    driveControls(leftFront, leftRear, rightFront, rightRear);}
                slideOne.setTargetPosition(desiredPos);
                slideTwo.setTargetPosition(desiredPos);
                slideOne.setPower(ARM_POWER);
                slideTwo.setPower(ARM_POWER);
                slideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while ((slideOne.isBusy()) && (slideTwo.isBusy())){
                    driveControls(leftFront, leftRear, rightFront, rightRear);}
                slideOne.setPower(0);
                slideTwo.setPower(0);
                liftWrist.setPosition(position4);
                position4 = .77;
                for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(200); stop>System.nanoTime();) {
                    driveControls(leftFront, leftRear, rightFront, rightRear);}
                spinOne.setPosition(position1);
                spinTwo.setPosition(position2);
                for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(500); stop>System.nanoTime();) {
                    driveControls(leftFront, leftRear, rightFront, rightRear);}
                armRote.setPosition(position3);
                for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(400); stop>System.nanoTime();) {
                    driveControls(leftFront, leftRear, rightFront, rightRear);}
                liftWrist.setPosition(position4);
                for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(200); stop>System.nanoTime();) {
                    driveControls(leftFront, leftRear, rightFront, rightRear);}
                position1 = .92;
                position2 = .92;
                spinOne.setPosition(position1);
                spinTwo.setPosition(position2);


            }
            //High
            if (gamepad1.y){
                high = true;
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
                high = false;
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
                high = false;
                position5 = 0;
                position1 = .48;
                position2 = .48;
                position3 = .83;
                position4 = .86;
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
            //ground pos
            if ((gamepad1.b)&&((slideOne.getCurrentPosition()>5)||(slideTwo.getCurrentPosition()>5))){
                if (high == false) {
                    position1 = .84;
                    position2 = .84;
                    position3 = .15;
                    position4 = .38;
                    position5 = 0;
                    desiredPos = 5;
                    armGrip.setPosition(position5);
                    spinOne.setPosition(position1);
                    spinTwo.setPosition(position2);
                    for (long stop = System.nanoTime() + TimeUnit.MILLISECONDS.toNanos(400); stop > System.nanoTime(); ) {
                        driveControls(leftFront, leftRear, rightFront, rightRear);
                        safetyCheck(rangeClaw);
                    }
                    liftWrist.setPosition(position4);
                    for (long stop = System.nanoTime() + TimeUnit.MILLISECONDS.toNanos(300); stop > System.nanoTime(); ) {
                        driveControls(leftFront, leftRear, rightFront, rightRear);
                        safetyCheck(rangeClaw);
                    }
                    position4 = .60;
                    armRote.setPosition(position3);
                    for (long stop = System.nanoTime() + TimeUnit.MILLISECONDS.toNanos(550); stop > System.nanoTime(); ) {
                        driveControls(leftFront, leftRear, rightFront, rightRear);
                        safetyCheck(rangeClaw);
                    }
                    position1 = .95;
                    position2 = .95;
                    liftWrist.setPosition(position4);
                    for (long stop = System.nanoTime() + TimeUnit.MILLISECONDS.toNanos(700); stop > System.nanoTime(); ) {
                        driveControls(leftFront, leftRear, rightFront, rightRear);
                        safetyCheck(rangeClaw);
                    }
                    spinOne.setPosition(position1);
                    spinTwo.setPosition(position2);
                    slideOne.setTargetPosition(desiredPos);
                    slideTwo.setTargetPosition(desiredPos);
                    slideOne.setPower(ARM_POWER);
                    slideTwo.setPower(ARM_POWER);
                    slideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while ((slideOne.isBusy()) && (slideTwo.isBusy())) {
                        driveControls(leftFront, leftRear, rightFront, rightRear);
                        safetyCheck(rangeClaw);
                    }
                    movingDown = false;
                    slideOne.setPower(0);
                    slideTwo.setPower(0);
                    position4 = .6;
                    liftWrist.setPosition(position4);
                    position5 = .18;
                    armGrip.setPosition(position5);
                }
                else if(high){
                    position1 = .95;
                    position2 = .95;
                    position3 = .15;
                    position4 = .60;
                    position5 = 0;
                    desiredPos = 5;
                    armGrip.setPosition(position5);
                    armRote.setPosition(position3);
                    for (long stop = System.nanoTime() + TimeUnit.MILLISECONDS.toNanos(400); stop > System.nanoTime(); ) {
                        driveControls(leftFront, leftRear, rightFront, rightRear);
                        safetyCheck(rangeClaw);
                    }
                    spinOne.setPosition(position1);
                    spinTwo.setPosition(position2);
                    for (long stop = System.nanoTime() + TimeUnit.MILLISECONDS.toNanos(400); stop > System.nanoTime(); ) {
                        driveControls(leftFront, leftRear, rightFront, rightRear);
                        safetyCheck(rangeClaw);
                    }
                    liftWrist.setPosition(position4);
                    for (long stop = System.nanoTime() + TimeUnit.MILLISECONDS.toNanos(800); stop > System.nanoTime(); ) {
                        driveControls(leftFront, leftRear, rightFront, rightRear);
                        safetyCheck(rangeClaw);
                    }
                    slideOne.setTargetPosition(desiredPos);
                    slideTwo.setTargetPosition(desiredPos);
                    slideOne.setPower(ARM_POWER);
                    slideTwo.setPower(ARM_POWER);
                    slideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while ((slideOne.isBusy()) && (slideTwo.isBusy())) {
                        driveControls(leftFront, leftRear, rightFront, rightRear);
                        safetyCheck(rangeClaw);
                    }
                    movingDown = false;
                    slideOne.setPower(0);
                    slideTwo.setPower(0);
                    position4 = .6;
                    liftWrist.setPosition(position4);
                    position5 = .18;
                    armGrip.setPosition(position5);
                }
                {
                    //control for high junction
                }
            }
            */
            // Display the current value
            telemetry.addData("spinOne Servo Position", "%5.2f", spinOne.getPosition());
            telemetry.addData("spinTwo Servo Position", "%5.2f", spinTwo.getPosition());
            telemetry.addData("armRote Servo Position", "%5.2f", armRote.getPosition());
            telemetry.addData("liftWrist Servo Position", "%5.2f", liftWrist.getPosition());
            telemetry.addData("armGrip Servo Position", "%5.2f", armGrip.getPosition());
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

    private void safetyCheck(double rangeClaw) {
        if (rangeClaw < 0)//a number)
        {
            spinOne.setPosition(spinOne.getPosition());
            spinTwo.setPosition(spinTwo.getPosition());
            armRote.setPosition(armRote.getPosition());
            liftWrist.setPosition(liftWrist.getPosition());
            armGrip.setPosition(armGrip.getPosition());
            slideOne.setPower(0);
            slideTwo.setPower(0);
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
