package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.concurrent.TimeUnit;


public class ArmControl {
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

    long  claw_time = 2000;
    long  wrist_time = 2000;
    long  spin_time = 2000;
    long  rotate_time = 2000;
    int START_POS = 5;
    int STOW_POS = 1400;
    int LOW_POS = 1850;   //2090
    int MED_POS = 3560;   //3681
    int HIGH_POS = 1550;  //1738
    boolean high = false;
    boolean stow = false;
    boolean ready = false;
    boolean intake = true;
    boolean IsDriverControl;
    OpMode opMode;
    public static final double ARM_POWER    =  1 ;

    public ArmControl(boolean isDriverControl, OpMode opMode) {
        this.IsDriverControl = isDriverControl;
        this.opMode = opMode;
    }

    public void Init (HardwareMap hardwareMap) {
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

        spinOne.setDirection(Servo.Direction.FORWARD);
        spinTwo.setDirection(Servo.Direction.REVERSE);
        armRote.setDirection(Servo.Direction.FORWARD);
        liftWrist.setDirection(Servo.Direction.FORWARD);
        armGrip.setDirection(Servo.Direction.FORWARD);
        slideOne.setDirection(DcMotor.Direction.FORWARD);
        slideTwo.setDirection(DcMotor.Direction.FORWARD);
        slideOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        StartPosition();
    }

    public void StartPosition() {
        armGrip.setPosition(0);
        SpecialSleep(null, 300);
        spinOne.setPosition(.95);
        spinTwo.setPosition(.95);
        armRote.setPosition(.11);
        liftWrist.setPosition(.6);
    }

    public void GrabFromStack(SampleMecanumDrive drive) {}

    public void GoToHigh(SampleMecanumDrive drive) {
        high = true;        // set position variable for return
        //************************************************************
        // verify claw is closed
        //************************************************************
        armGrip.setPosition(0);
        //************************************************************
        // raise slides to high junction delivery height
        //************************************************************
        slideOne.setTargetPosition(HIGH_POS);
        slideTwo.setTargetPosition(HIGH_POS);
        slideOne.setPower(ARM_POWER);
        slideTwo.setPower(ARM_POWER);
        WaitForSlides(drive);
        slideOne.setPower(0);
        slideTwo.setPower(0);
        //************************************************************
        // swinging arm to high junction position
        //************************************************************
        spinOne.setPosition(.11);
        spinTwo.setPosition(.11);
        //************************************************************
        // wrist to delivery angle (high)
        //************************************************************
        liftWrist.setPosition(.13);
        // set stow variable to false
        stow = false;
        // set ready for delivery variable to true
        ready = true;
    }

    public void GoToHigh() {}

    public void GoToMedium() {}

    public void GoToLow() {}

    public void ReturnFromHigh() {}

    public void ReturnFromLowMedium() {}

    private void driveControls() {
        double y = opMode.gamepad2.left_stick_y;
        double x = -opMode.gamepad2.left_stick_x * 1.1;
        double rx = opMode.gamepad2.right_stick_x;

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

    private void WaitForTrajectoryToFinish(SampleMecanumDrive drive) {
        while(drive.isBusy()) {
            drive.update();
            if (IsDriverControl) {
                driveControls();
            }
        }
    }

    private void WaitForSlides(SampleMecanumDrive drive) {
        while ((slideOne.isBusy()) || (slideTwo.isBusy())) {
            drive.update();
            if (IsDriverControl) {
                driveControls();
            }
        }
    }

    private void SpecialSleep(SampleMecanumDrive drive, long milliseconds) {
        for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(milliseconds); stop>System.nanoTime();) {
            drive.update();
            if (IsDriverControl) {
                driveControls();
            }
        }
    }
}

