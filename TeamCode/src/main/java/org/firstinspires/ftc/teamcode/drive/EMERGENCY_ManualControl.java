package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

// @Disabled
@TeleOp(name = "EMERGENCY MANUAL CONTROL")

public class EMERGENCY_ManualControl extends LinearOpMode {

    static final double SCALE       = 0.01;     // Joystick scaling for servo increment value
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

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
    double  position1 = .95; //spinOne, left stick y
    double  position2 = .95; //spinTwo
    double  position3 = .11; //armRote, right stick x
    double  position4 = .6; //liftWrist, right stick y
    double  position5 = 0; //armGrip, bumpers

    @Override
    public void runOpMode() {

        // Servo names in hardware map on Control or Expansion Hubs
        spinOne = hardwareMap.get(Servo.class, "spinOne");
        spinTwo = hardwareMap.get(Servo.class, "spinTwo");
        armRote = hardwareMap.get(Servo.class, "armRote");
        liftWrist = hardwareMap.get(Servo.class, "liftWrist");
        armGrip = hardwareMap.get(Servo.class, "armGrip");
        slideOne = hardwareMap.get(DcMotor.class, "slideOne");
        slideTwo = hardwareMap.get(DcMotor.class, "slideTwo");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        // Slide Init
        slideOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideOne.setDirection(DcMotor.Direction.FORWARD);
        slideTwo.setDirection(DcMotor.Direction.FORWARD);
        slideOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Chassis Motor Config
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // Servo direction
        spinOne.setDirection(Servo.Direction.FORWARD);
        spinTwo.setDirection(Servo.Direction.REVERSE);
        armRote.setDirection(Servo.Direction.FORWARD);
        liftWrist.setDirection(Servo.Direction.FORWARD);
        armGrip.setDirection(Servo.Direction.FORWARD);

        // Wait for the start button
        telemetry.addData(">", "Press Start to move Servos with left joystick and triggers." );
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){

            driveControls(leftFront, leftRear, rightFront, rightRear);

            if (gamepad1.left_bumper) {
                position5 = .18;
            }
            if (gamepad1.right_bumper) {
                position5 = 0;
            }

            if(gamepad1.dpad_down){ //slides down
                slideOne.setPower(-1);
                slideTwo.setPower(-1);
            }
            else if(gamepad1.dpad_up){ //slides up
                slideOne.setPower(1);
                slideTwo.setPower(1);
            }
            else {
                slideOne.setPower(0);
                slideTwo.setPower(0);
            }

            if (gamepad1.right_stick_y != 0) { //swing
                position1 += gamepad1.right_stick_y * SCALE;
                position2 += gamepad1.right_stick_y * SCALE;
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

            if (gamepad1.right_stick_x != 0) { //armRotate
                position3 += gamepad1.right_stick_x * SCALE;
                if (position3 >= MAX_POS) {
                    position3 = MAX_POS;
                }
                if (position3 <= MIN_POS) {
                    position3 = MIN_POS;
                }
            }

            if (gamepad1.left_stick_y != 0) { //wrist
                position4 += gamepad1.left_stick_y * SCALE;
                if (position4 >= MAX_POS) {
                    position4 = MAX_POS;
                }
                if (position4 <= MIN_POS) {
                    position4 = MIN_POS;
                }
            }

            // Display the current value
            telemetry.addData("spinOne Servo Position", "%5.2f", spinOne.getPosition());
            telemetry.addData("spinTwo Servo Position", "%5.2f", spinTwo.getPosition());
            telemetry.addData("armRote Servo Position", "%5.2f", armRote.getPosition());
            telemetry.addData("liftWrist Servo Position", "%5.2f", liftWrist.getPosition());
            telemetry.addData("armGrip Servo Position", "%5.2f", armGrip.getPosition());
            telemetry.addData("slideOnePos: ", slideOne.getCurrentPosition());
            telemetry.addData("slideTwoPos: ", slideTwo.getCurrentPosition());
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
