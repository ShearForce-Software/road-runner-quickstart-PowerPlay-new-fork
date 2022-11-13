package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
    double  position1 = 0.8;
    double  position2 = 0.8;
    double  position3 = 0.13;
    double  position4 = 0.65;
    double  position5 = 0.0;
    int desiredPos = 0;
    int runToPos = 0;
    public static final double ARM_POWER    =  0.50 ;

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
        slideTwo.setDirection(DcMotor.Direction.REVERSE);

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
            if (gamepad1.left_trigger != 0.0) {
                position4 += gamepad1.left_trigger * SCALE;
                if (position4 >= MAX_POS) {
                    position4 = MAX_POS;
                }
                if (position4 <= MIN_POS) {
                    position4 = MIN_POS;
                }
            }
            if (gamepad1.right_trigger != 0.0){
                position4 -= gamepad1.right_trigger * SCALE;
                if (position4 >= MAX_POS) {
                    position4 = MAX_POS;
                }
                if (position4 <= MIN_POS) {
                    position4 = MIN_POS;
                }
            }
            if (gamepad1.left_bumper) {
                position5 = .5;
            }
            if (gamepad1.right_bumper) {
                position5 = 0;
            }
            double rangeRear = rearDistance.getDistance(DistanceUnit.CM);
            double rangeClaw = clawDistance.getDistance(DistanceUnit.CM);
            double rangeFront = frontDistance.getDistance(DistanceUnit.CM);

            if (rangeClaw < 2.75) {
                position5 = 0;
                position1 = .67;
                position2 = .67;
                desiredPos = 1065;
                slideOne.setTargetPosition(desiredPos);
                slideTwo.setTargetPosition(desiredPos);
                slideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideOne.setPower(ARM_POWER);
                slideTwo.setPower(ARM_POWER);

                armGrip.setPosition(position5);
                sleep(3000);
                spinOne.setPosition(position1);
                spinTwo.setPosition(position2);
            }
            //High
            if (gamepad1.y){
                position1 = 0;
                position2 = 0;
                position3 = .83;
                position4 = .22;
                desiredPos = 1529;
                slideOne.setTargetPosition(desiredPos);
                slideTwo.setTargetPosition(desiredPos);
                slideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideOne.setPower(ARM_POWER);
                slideTwo.setPower(ARM_POWER);

                spinOne.setPosition(position1);
                spinTwo.setPosition(position2);
                armRote.setPosition(position3);
                sleep(6000);
                liftWrist.setPosition(position4);
            }
            //Medium
            if (gamepad1.x){
                position1 = .68;
                position2 = .68;
                position3 = .83;
                position4 = .94;
                desiredPos = 3823;
                slideOne.setTargetPosition(desiredPos);
                slideTwo.setTargetPosition(desiredPos);
                slideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideOne.setPower(ARM_POWER);
                slideTwo.setPower(ARM_POWER);

                spinOne.setPosition(position1);
                spinTwo.setPosition(position2);
                armRote.setPosition(position3);
                sleep(6000);
                liftWrist.setPosition(position4);
            }
            //Low
            if (gamepad1.a){
                position1 = .66;
                position2 = .66;
                position3 = .83;
                position4 = .94;
                desiredPos = 1909;
                slideOne.setTargetPosition(desiredPos);
                slideTwo.setTargetPosition(desiredPos);
                slideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideOne.setPower(ARM_POWER);
                slideTwo.setPower(ARM_POWER);

                spinOne.setPosition(position1);
                spinTwo.setPosition(position2);
                armRote.setPosition(position3);
                sleep(6000);
                liftWrist.setPosition(position4);
            }
            // Display the current value
            telemetry.addData("spinOne Servo Position", "%5.2f", position1);
            telemetry.addData("spinTwo Servo Position", "%5.2f", position2);
            telemetry.addData("armRote Servo Position", "%5.2f", position3);
            telemetry.addData("liftWrist Servo Position", "%5.2f", position4);
            telemetry.addData("armGrip Servo Position", "%5.2f", position5);
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
