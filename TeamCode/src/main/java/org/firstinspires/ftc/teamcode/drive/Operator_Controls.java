package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Operator Controls")

public class Operator_Controls extends LinearOpMode {

    static final double SCALE       = 0.01;
    static final double MAX_POS     =  1.0;
    static final double MIN_POS     =  0.0;

    // Define class members
    Servo  spinOne;
    Servo  spinTwo;
    Servo  armRote;
    Servo  liftWrist;
    Servo  armGrip;
    DcMotor slideOne;
    DcMotor slideTwo;
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

    @Override
    public void runOpMode() {

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

        // Wait for the start button
        telemetry.addData(">", "Press Start to move Servos with left joystick and triggers." );
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){

            if (gamepad1.left_stick_y != 0) {
                position1 += gamepad1.left_stick_y * SCALE;
                position2 += gamepad1.left_stick_y * SCALE;
                if (position1 >= MAX_POS) {
                    position1 = MAX_POS;
                }
                if (position1 <= MIN_POS) {
                    position1 = MIN_POS;
                }
                if (position2 >= MAX_POS) {
                    position2 = MAX_POS;
                }
                if (position2 <= MIN_POS) {
                    position2 = MIN_POS;
                }
            }
            if ((gamepad1.right_stick_x != 0) && (gamepad1.right_stick_button==false)) { //armRotate
                position3 += gamepad1.right_stick_x * SCALE;
                if (position3 >= MAX_POS) {
                    position3 = MAX_POS;
                }
                if (position3 <= MIN_POS) {
                    position3 = MIN_POS;
                }
            }
            if (gamepad1.left_bumper) {
                position5 = .15;
            }
            if (gamepad1.right_bumper) {
                position5 = 0;
            }
            double rangeRear = rearDistance.getDistance(DistanceUnit.CM);
            double rangeClaw = clawDistance.getDistance(DistanceUnit.CM);
            double rangeFront = frontDistance.getDistance(DistanceUnit.CM);

            if ((((rangeClaw < 2.75) && (position5 == .15)) && (slideOne.getCurrentPosition() <= 100) && (slideTwo.getCurrentPosition() <= 100))) {
                position5 = 0;
                position1 = .80;
                position2 = .80;
                desiredPos = 1244;
                armGrip.setPosition(position5);
                sleep(200);
                slideOne.setTargetPosition(desiredPos);
                slideTwo.setTargetPosition(desiredPos);
                slideOne.setPower(ARM_POWER);
                slideTwo.setPower(ARM_POWER);
                slideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while ((slideOne.isBusy()) && (slideTwo.isBusy())){
                }
                slideOne.setPower(0);
                slideTwo.setPower(0);


                spinOne.setPosition(position1);
                spinTwo.setPosition(position2);
            }
            //High
            if (gamepad1.y){
                position1 = .11;
                position2 = .11;
                position3 = .83;
                position4 = .13;
                desiredPos = 1969;
                slideOne.setTargetPosition(desiredPos);
                slideTwo.setTargetPosition(desiredPos);
                slideOne.setPower(ARM_POWER);
                slideTwo.setPower(ARM_POWER);
                slideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while ((slideOne.isBusy()) && (slideTwo.isBusy())){
                }
                slideOne.setPower(0);
                slideTwo.setPower(0);

                spinOne.setPosition(position1);
                spinTwo.setPosition(position2);
                sleep(1000);
                armRote.setPosition(position3);
                liftWrist.setPosition(position4);
            }
            //Medium
            if (gamepad1.x){
                position1 = .83;
                position2 = .83;
                position3 = .83;
                position4 = .86;
                desiredPos = 3777;
                slideOne.setTargetPosition(desiredPos);
                slideTwo.setTargetPosition(desiredPos);
                slideOne.setPower(ARM_POWER);
                slideTwo.setPower(ARM_POWER);
                slideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while ((slideOne.isBusy()) && (slideTwo.isBusy())){
                }
                slideOne.setPower(0);
                slideTwo.setPower(0);

                spinOne.setPosition(position1);
                spinTwo.setPosition(position2);
                sleep(1000);
                armRote.setPosition(position3);
                liftWrist.setPosition(position4);
            }
            //Low
            if (gamepad1.a){
                position1 = .82;
                position2 = .82;
                position3 = .83;
                position4 = .82;
                desiredPos = 2176;
                slideOne.setTargetPosition(desiredPos);
                slideTwo.setTargetPosition(desiredPos);
                slideOne.setPower(ARM_POWER);
                slideTwo.setPower(ARM_POWER);
                slideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while ((slideOne.isBusy()) && (slideTwo.isBusy())){
                }
                slideOne.setPower(0);
                slideTwo.setPower(0);

                spinOne.setPosition(position1);
                spinTwo.setPosition(position2);
                sleep(1000);
                armRote.setPosition(position3);
                liftWrist.setPosition(position4);
            }
            if (gamepad1.b){
                position1 = .95;
                position2 = .95;
                position3 = .13;
                position4 = .74;
                position5 = 0;
                desiredPos = 5;
                armGrip.setPosition(position5);
                liftWrist.setPosition(position4);
                armRote.setPosition(position3);
                sleep(500);
                spinOne.setPosition(position1);
                spinTwo.setPosition(position2);
                slideOne.setTargetPosition(desiredPos);
                slideTwo.setTargetPosition(desiredPos);
                slideOne.setPower(ARM_POWER);
                slideTwo.setPower(ARM_POWER);
                slideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while ((slideOne.isBusy()) && (slideTwo.isBusy())){
                }
                slideOne.setPower(0);
                slideTwo.setPower(0);
                position4 = .6;
                liftWrist.setPosition(position4);
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
}
