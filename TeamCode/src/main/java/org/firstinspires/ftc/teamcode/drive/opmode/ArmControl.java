package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.hardware.bosch.BNO055IMU;
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

import java.sql.Savepoint;
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
    BNO055IMU imu;

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
    boolean IsFieldCentric;
    OpMode opMode;
    public static final double ARM_POWER    =  1 ;

    public ArmControl(boolean isDriverControl, boolean isFieldCentric, OpMode opMode) {
        this.IsDriverControl = isDriverControl;
        this.IsFieldCentric = isFieldCentric;
        this.opMode = opMode;
    }

    public void Init (HardwareMap hardwareMap) {
        // Distance Sensors
        rearDistance = hardwareMap.get(DistanceSensor.class, "rearDistance");
        clawDistance = hardwareMap.get(DistanceSensor.class, "clawDistance");
        frontDistance = hardwareMap.get(DistanceSensor.class, "frontDistance");

        // Arm Servos
        spinOne = hardwareMap.get(Servo.class, "spinOne");
        spinTwo = hardwareMap.get(Servo.class, "spinTwo");
        armRote = hardwareMap.get(Servo.class, "armRote");
        liftWrist = hardwareMap.get(Servo.class, "liftWrist");
        armGrip = hardwareMap.get(Servo.class, "armGrip");

        // Slide Motors
        slideOne = hardwareMap.get(DcMotor.class, "slideOne");
        slideTwo = hardwareMap.get(DcMotor.class, "slideTwo");

        // Chassis Motors
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightRear = hardwareMap.dcMotor.get("rightRear");

        // Servo Direction
        spinOne.setDirection(Servo.Direction.FORWARD);
        spinTwo.setDirection(Servo.Direction.REVERSE);
        armRote.setDirection(Servo.Direction.FORWARD);
        liftWrist.setDirection(Servo.Direction.FORWARD);
        armGrip.setDirection(Servo.Direction.FORWARD);

        // Configure Motors
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

        // IMU Initialize
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Move Arm Servos into Start Position
        StartPosition();
    }

    public void StartPosition() {
        spinOne.setPosition(.95);
        spinTwo.setPosition(.95);
        armRote.setPosition(.11);
        liftWrist.setPosition(.6);
        armGrip.setPosition(0);
    }

    public void GrabConeAndStowFromIntake(SampleMecanumDrive drive) {

        //************************************************************
        // close claw when cone is detected by distance sensor
        //************************************************************
        armGrip.setPosition(0); // close claw
        claw_time = 300;        // claw close time
        SpecialSleep(drive, claw_time);

        //************************************************************
        // raise slide to cone stow position height
        //************************************************************
        slideOne.setTargetPosition(STOW_POS);
        slideTwo.setTargetPosition(STOW_POS);
        slideOne.setPower(ARM_POWER);
        slideTwo.setPower(ARM_POWER);
        WaitForSlides(drive);           // wait for slides before moving arm
        slideOne.setPower(0);
        slideTwo.setPower(0);

        //************************************************************
        // straighten wrist before rotating 180 degrees
        //************************************************************
        liftWrist.setPosition(.35);     // straighten wrist
        wrist_time = 200;               // wrist straighten time
        SpecialSleep(drive, wrist_time);

        //************************************************************
        // rotate arm 180 degrees to flip cone
        //************************************************************
        armRote.setPosition(.82); // rotate arm 180 degrees
        rotate_time = 600;        // rotate arm time
        SpecialSleep(drive, rotate_time);

        //************************************************************
        // spin arm to cone stow rotate position
        //************************************************************
        spinOne.setPosition(.84); // spin arm to cone stow rotate position
        spinTwo.setPosition(.84); // spin arm to cone stow rotate position

        stow = true;              // set stow variable to true
        intake = false;           // set input variable to false
    }

    public void GrabConeAndStowFromStack(SampleMecanumDrive drive) {}

    public void GoToHigh(SampleMecanumDrive drive) {

        high = true;            // set position variable for return

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

        //************************************************************
        // spin arm to high junction position
        //************************************************************
        spinOne.setPosition(.11);
        spinTwo.setPosition(.11);

        //************************************************************
        // wrist to delivery angle (high)
        //************************************************************
        liftWrist.setPosition(.13);

        WaitForSlides(drive);   // wait for slides to finish
        slideOne.setPower(0);   // slide power to 0
        slideTwo.setPower(0);   // slide power to 0
        stow = false;           // set stow variable to false
        ready = true;           // set ready for delivery variable to true
    }

    public void GoToMedium(SampleMecanumDrive drive) {

        high = false;       // set position variable for return

        //************************************************************
        // verify claw is closed
        //************************************************************
        armGrip.setPosition(0);

        //************************************************************
        // raise slides to medium junction delivery height
        //************************************************************
        slideOne.setTargetPosition(MED_POS);
        slideTwo.setTargetPosition(MED_POS);
        slideOne.setPower(ARM_POWER);
        slideTwo.setPower(ARM_POWER);

        //************************************************************
        // spin arm to medium junction position
        //************************************************************
        spinOne.setPosition(0.83);
        spinTwo.setPosition(0.83);

        //************************************************************
        // wrist to delivery angle (Medium)
        //************************************************************
        liftWrist.setPosition(0.86); // wrist to delivery angle

        WaitForSlides(drive);   // wait for slides to finish
        slideOne.setPower(0);   // slide power to 0
        slideTwo.setPower(0);   // slide power to 0
        stow = false;           // set stow variable to false
        ready = true;           // set ready for delivery variable to true
    }

    public void GoToLow(SampleMecanumDrive drive) {

        high = false;       // set position variable for return

        //************************************************************
        // verify claw is closed
        //************************************************************
        armGrip.setPosition(0);

        //************************************************************
        // raise slides to low junction delivery height
        //************************************************************
        slideOne.setTargetPosition(LOW_POS);
        slideTwo.setTargetPosition(LOW_POS);
        slideOne.setPower(ARM_POWER);
        slideTwo.setPower(ARM_POWER);

        //************************************************************
        // spin arm to low junction position
        //************************************************************
        spinOne.setPosition(0.83);
        spinTwo.setPosition(0.83);

        //************************************************************
        // wrist to delivery angle (Low)
        //************************************************************
        liftWrist.setPosition(0.86); // wrist to delivery angle

        WaitForSlides(drive);   // wait for slides to finish
        slideOne.setPower(0);   // slide power to 0
        slideTwo.setPower(0);   // slide power to 0
        stow = false;           // set stow variable to false
        ready = true;           // set ready for delivery variable to true
    }

    public void GoToGround(SampleMecanumDrive drive) {}

    public void GoToClawStow(SampleMecanumDrive drive) {}

    public void ReturnFromHighToIntake(SampleMecanumDrive drive) {

        //************************************************************
        // Close claw
        //************************************************************
        armGrip.setPosition(0);

        //************************************************************
        // rotate arm 180 degrees (so gripper is backwards)
        //************************************************************
        armRote.setPosition(0.11);
        rotate_time = 550;
        SpecialSleep(drive, rotate_time);

        //************************************************************
        // spin arm to cone pickup position
        //************************************************************
        spinOne.setPosition(0.95);
        spinTwo.setPosition(0.95);
        spin_time = 700;
        SpecialSleep(drive, spin_time);

        //************************************************************
        // wrist to cone pickup position
        //************************************************************
        liftWrist.setPosition(0.6);
        wrist_time = 450;
        SpecialSleep(drive, wrist_time);

        //************************************************************
        // lower slides to cone intake height
        //************************************************************
        slideOne.setTargetPosition(START_POS);
        slideTwo.setTargetPosition(START_POS);
        slideOne.setPower(ARM_POWER);
        slideTwo.setPower(ARM_POWER);
        WaitForSlides(drive);
        slideOne.setPower(0);
        slideTwo.setPower(0);

        //************************************************************
        // Open claw
        //************************************************************
        armGrip.setPosition(0.18);
        intake = true;
    }

    public void ReturnFromLowMediumToIntake(SampleMecanumDrive drive) {

        //************************************************************
        // Close claw
        //************************************************************
        armGrip.setPosition(0);
        claw_time = 200;
        SpecialSleep(drive,claw_time);

        //************************************************************
        // spin arm to safe return position
        //************************************************************
        spinOne.setPosition(0.95);
        spinTwo.setPosition(0.95);
        spin_time = 300;
        SpecialSleep(drive,spin_time);

        //************************************************************
        // straighten wrist
        //************************************************************
        liftWrist.setPosition(0.35);
        wrist_time = 300;
        SpecialSleep(drive,wrist_time);

        //************************************************************
        // rotate arm 180 degrees
        //************************************************************
        armRote.setPosition(0.11);
        rotate_time = 550;
        SpecialSleep(drive,rotate_time);

        //************************************************************
        // wrist to cone pickup position
        //************************************************************
        liftWrist.setPosition(0.6);
        wrist_time = 200;
        SpecialSleep(drive,wrist_time);

        //************************************************************
        // lower slides to cone intake height
        //************************************************************
        slideOne.setTargetPosition(START_POS);
        slideTwo.setTargetPosition(START_POS);
        slideOne.setPower(ARM_POWER);
        slideTwo.setPower(ARM_POWER);
        WaitForSlides(drive);
        slideOne.setPower(0);
        slideTwo.setPower(0);

        //************************************************************
        // Open claw
        //************************************************************
        armGrip.setPosition(0.18);
        intake = true;
    }

    public void ReturnFromGroundToIntake(SampleMecanumDrive drive) {
        // Future movement to return from Ground Junction
    }

    public void ReturnFromHighToStack(SampleMecanumDrive drive) {
        // same as ReturnFromHighToIntake except stops at 5 high cone encoder height
        // then jumps to GrabConeAndStowFromStack
    }

    public void ReturnFromClawStowToIntake(SampleMecanumDrive drive) {
        // Driver button push to return claw to intake
    }

    private void driveControlsRobotCentric() {
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

    private void driveControlsFieldCentric() {
        double y = -opMode.gamepad2.left_stick_y;
        double x = opMode.gamepad2.left_stick_x * 1.1;
        double rx = opMode.gamepad2.right_stick_x;

        double botHeading = -imu.getAngularOrientation().firstAngle;

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);
    }

    private void WaitForTrajectoryToFinish(SampleMecanumDrive drive) {
        while(drive.isBusy()) {
            if(drive != null) drive.update();
            if (IsDriverControl) {
                if(IsFieldCentric) driveControlsFieldCentric();
                if(!IsFieldCentric) driveControlsRobotCentric();
            }
        }
    }

    private void WaitForSlides(SampleMecanumDrive drive) {
        while ((slideOne.isBusy()) || (slideTwo.isBusy())) {
            if(drive != null) drive.update();
            if (IsDriverControl) {
                if(IsFieldCentric) driveControlsFieldCentric();
                if(!IsFieldCentric) driveControlsRobotCentric();
            }
        }
    }

    private void SpecialSleep(SampleMecanumDrive drive, long milliseconds) {
        for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(milliseconds); stop>System.nanoTime();) {
            if(drive != null) drive.update();
            if (IsDriverControl) {
                if(IsFieldCentric) driveControlsFieldCentric();
                if(!IsFieldCentric) driveControlsRobotCentric();
            }
        }
    }
}

