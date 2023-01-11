package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    BNO055IMU imu;

    double rangeRear;
    double rangeClaw;
    double rangeFront;

    int START_POS = 10; //5
    int STOW_POS = 1080; //1400
    int LOW_POS = 1250;   //1850
    int MED_POS = 2370;   //3560
    int HIGH_POS = 1080;  //1550     900
    int STACK_POS = 550; //1100
    public boolean high = false;
    public boolean stow = false;
    public boolean readyToDrop = false;
    public boolean intake = true;
    public boolean coneStack = false;
    boolean IsDriverControl;
    boolean IsFieldCentric;
    LinearOpMode opMode;
    public static final double ARM_POWER =  1 ;
    public ArmControl(boolean isDriverControl, boolean isFieldCentric, LinearOpMode opMode) {
        this.IsDriverControl = isDriverControl;
        this.IsFieldCentric = isFieldCentric;
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
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
//        leftRear = hardwareMap.dcMotor.get("leftRear");
//        rightFront = hardwareMap.dcMotor.get("rightFront");
//        rightRear = hardwareMap.dcMotor.get("rightRear");

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
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void StartPosition(SampleMecanumDrive drive) {
        slideOne.setTargetPosition(0);
        slideTwo.setTargetPosition(0);
        slideOne.setPower(ARM_POWER);
        slideTwo.setPower(ARM_POWER);
        WaitForSlides(drive);
        slideOne.setPower(0);
        slideTwo.setPower(0);
        armGrip.setPosition(0);
        SpecialSleep(null, 300);
        spinOne.setPosition(.97);
        spinTwo.setPosition(.97);
        armRote.setPosition(.11);
        liftWrist.setPosition(.6);
    }

    public void GoToHigh(SampleMecanumDrive drive) {
        high = true;        // set position variable for return
        //************************************************************
        // verify claw is closed
        //************************************************************
        armGrip.setPosition(0);
        //************************************************************
        // swinging arm to high junction position
        //************************************************************
        spinOne.setPosition(.14);
        spinTwo.setPosition(.14);
        SpecialSleep(drive, 300);
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
        // wrist to delivery angle (high)
        //************************************************************
        liftWrist.setPosition(.13);
        // set stow variable to false
        stow = false;
        // set ready for delivery variable to true
        readyToDrop = true;
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
        WaitForSlides(drive);
        slideOne.setPower(0);
        slideTwo.setPower(0);
        //************************************************************
        // spin arm to medium junction position
        //************************************************************
        spinOne.setPosition(0.83);
        spinTwo.setPosition(0.83);
        //************************************************************
        // wrist to delivery angle (Medium)
        //************************************************************
        liftWrist.setPosition(0.86); // wrist to delivery angle
        // set stow variable to false
        stow = false;
        // set ready for delivery variable to true
        readyToDrop = true;}

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
        WaitForSlides(drive);
        slideOne.setPower(0);
        slideTwo.setPower(0);
        //************************************************************
        // spin arm to low junction position
        //************************************************************
        spinOne.setPosition(0.83);
        spinTwo.setPosition(0.83);
        //************************************************************
        // wrist to delivery angle (Low)
        //************************************************************
        liftWrist.setPosition(0.86); // wrist to delivery angle
        // set stow variable to false
        stow = false;
        // set ready for delivery variable to true
        readyToDrop = true;
    }

    public void ReturnFromLowMedium(SampleMecanumDrive drive) {
        armGrip.setPosition(.18);
        SpecialSleep(drive, 100);
        //************************************************************
        // Close claw
        //************************************************************
        armGrip.setPosition(0);
        SpecialSleep(drive, 200);
        //************************************************************
        // spin arm to safe return position
        //************************************************************
        spinOne.setPosition(0.95);
        spinTwo.setPosition(0.95);
        SpecialSleep(drive, 300);
        //************************************************************
        // straighten wrist
        //************************************************************
        liftWrist.setPosition(0.35);
        SpecialSleep(drive, 300);
        //************************************************************
        // rotate arm 180 degrees
        //************************************************************
        armRote.setPosition(0.11);
        SpecialSleep(drive,550);
        //************************************************************
        // wrist to cone pickup position
        //************************************************************
        liftWrist.setPosition(0.6);
        SpecialSleep(drive, 200);
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

    public void ReturnFromHigh(SampleMecanumDrive drive) {
        armGrip.setPosition(.18);
        SpecialSleep(drive, 100);
        //************************************************************
        // Close claw
        //************************************************************
        armGrip.setPosition(0);
        //************************************************************
        // rotate arm 180 degrees (so gripper is backwards)
        //************************************************************
        armRote.setPosition(0.11);
        //************************************************************
        // spin arm to cone pickup position
        //************************************************************
        SpecialSleep(drive, 700);
        spinOne.setPosition(0.95);
        spinTwo.setPosition(0.95);
        //************************************************************
        // wrist to cone pickup position
        //************************************************************
        liftWrist.setPosition(0.6);
        SpecialSleep(drive, 600);
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

    public void ReadyToGrabFromStack(SampleMecanumDrive drive) {
        armGrip.setPosition(0); // Close claw
        armRote.setPosition(0.11); // rotate arm 180 degrees (so gripper is backwards)
        SpecialSleep(drive, 700);
        spinOne.setPosition(0.97); // spin arm to cone pickup position
        spinTwo.setPosition(0.97);
        liftWrist.setPosition(0.6); // wrist to cone pickup position
        SpecialSleep(drive, 600);
        slideOne.setTargetPosition(STACK_POS);
        slideTwo.setTargetPosition(STACK_POS);
        slideOne.setPower(ARM_POWER);
        slideTwo.setPower(ARM_POWER);
        armRote.setPosition(.13);
        liftWrist.setPosition(.6);
        SpecialSleep(drive, 500);
        armGrip.setPosition(.18);
    }

    public void GrabFromStack(SampleMecanumDrive drive) {
        armGrip.setPosition(0);
        SpecialSleep(drive, 200);
        slideOne.setPower(ARM_POWER);
        slideTwo.setPower(ARM_POWER);
        slideOne.setTargetPosition(1200);
        slideTwo.setTargetPosition(1200);
        WaitForSlides(drive);
        slideOne.setPower(0);
        slideTwo.setPower(0);
    }

    public void SafetyStow(SampleMecanumDrive drive) {
        stow = false;
        slideOne.setTargetPosition(10);
        slideTwo.setTargetPosition(10);
        slideOne.setPower(ARM_POWER);
        slideTwo.setPower(ARM_POWER);
        WaitForSlides(drive);
        slideOne.setPower(0);
        slideTwo.setPower(0);
        armGrip.setPosition(0);
        spinOne.setPosition(.88);
        spinTwo.setPosition(.88);
        armRote.setPosition(.11);
        liftWrist.setPosition(.75);
    }
//        coneStack = true;
//        while (coneStack) {
//            slideOne.setTargetPosition(10);
//            slideTwo.setTargetPosition(10);
//            slideOne.setPower(.3);
//            slideTwo.setPower(.3);
//            rangeClaw = clawDistance.getDistance(DistanceUnit.CM);
//            if (rangeClaw <= 2.75) {
//                coneStack = false;
//                slideOne.setPower(0);
//                slideTwo.setPower(0);
//                armGrip.setPosition(0);
//                SpecialSleep(drive, 200);
//                slideOne.setPower(ARM_POWER);
//                slideTwo.setPower(ARM_POWER);
//                slideOne.setTargetPosition(1200);
//                slideTwo.setTargetPosition(1200);
//                WaitForSlides(drive);
//                slideOne.setPower(0);
//                slideTwo.setPower(0);
//            }
//            else if ((slideOne.getCurrentPosition() <= 10) || (slideTwo.getCurrentPosition() <= 10)){
//                coneStack = false;

    public void StowCone(SampleMecanumDrive drive){
        //************************************************************
        // close claw when cone is detected by distance sensor
        //************************************************************
        // armGrip close position
        armGrip.setPosition(0); // close claw
        // wait for claw to close
        SpecialSleep(drive, 300);
        //************************************************************
        // raise slide to cone stow position height
        //************************************************************
        slideOne.setTargetPosition(STOW_POS);
        slideTwo.setTargetPosition(STOW_POS);
        slideOne.setPower(ARM_POWER);
        slideTwo.setPower(ARM_POWER);
        WaitForSlides(drive);
        slideOne.setPower(0);
        slideTwo.setPower(0);
        //************************************************************
        // straighten wrist before rotating 180 degrees
        //************************************************************
        // liftWrist straight position
        liftWrist.setPosition(.35);   // straighten wrist
        // wait for wrist to straighten
        SpecialSleep(drive, 200);
        //slideHeight();}
        //************************************************************
        // rotate arm 180 degrees to flip cone
        //************************************************************
        // armRote position
        armRote.setPosition(.82); // rotate arm 180 degrees
        // wait for arm to rotate
        SpecialSleep(drive, 600);
        //slideHeight();}
        //************************************************************
        // spin arm to cone stow rotate position
        //************************************************************
        spinOne.setPosition(.84); // spin arm to cone stow rotate position
        spinTwo.setPosition(.84); // spin arm to cone stow rotate position
        stow = true;
        // set input variable to false
        intake = false;
    }

    public void closeClaw(){
        armGrip.setPosition(0);
    }

    public void openClaw(){
        armGrip.setPosition(.18);
    }

    public void driveControlsRobotCentric() {
        double y = opMode.gamepad2.left_stick_y;
        double x = -opMode.gamepad2.left_stick_x * 1.1;
        double rx = opMode.gamepad2.right_stick_x;

        rangeRear = rearDistance.getDistance(DistanceUnit.CM);
        rangeClaw = clawDistance.getDistance(DistanceUnit.CM);
        rangeFront = frontDistance.getDistance(DistanceUnit.CM);

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

    public void driveControlsFieldCentric() {
        double y = -opMode.gamepad2.left_stick_y;
        double x = opMode.gamepad2.left_stick_x * 1.1;
        double rx = opMode.gamepad2.right_stick_x;

        rangeRear = rearDistance.getDistance(DistanceUnit.CM);
        rangeClaw = clawDistance.getDistance(DistanceUnit.CM);
        rangeFront = frontDistance.getDistance(DistanceUnit.CM);

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

    public void WaitForTrajectoryToFinish(SampleMecanumDrive drive) {
        while(opMode.opModeIsActive() && !opMode.isStopRequested() && drive.isBusy()) {
            if(drive != null){
                drive.update();
            }
            if (IsDriverControl) {
                if(IsFieldCentric) driveControlsFieldCentric();
                if(!IsFieldCentric) driveControlsRobotCentric();
            }
        }
    }

    public void WaitForSlides(SampleMecanumDrive drive) {
        while ((slideOne.isBusy()) && (slideTwo.isBusy()) && (!opMode.isStopRequested())) {
            if(drive != null) {
                drive.update();
//                Pose2d poseEstimate = drive.getPoseEstimate();
//                opMode.telemetry.addData("y", poseEstimate.getX());
//                opMode.telemetry.addData("x", poseEstimate.getY());
//                opMode.telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
//                opMode.telemetry.addData("front distance", rangeClaw);
//                opMode.telemetry.update();
            }
            if (IsDriverControl) {
                if(IsFieldCentric) driveControlsFieldCentric();
                if(!IsFieldCentric) driveControlsRobotCentric();
            }
        }
    }

    public void SpecialSleep(SampleMecanumDrive drive, long milliseconds) {
        for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(milliseconds); stop>System.nanoTime();) {
            if (!opMode.opModeIsActive() || opMode.isStopRequested() ) return;
            if(drive != null) {
                drive.update();
//                Pose2d poseEstimate = drive.getPoseEstimate();
//                opMode.telemetry.addData("y", poseEstimate.getX());
//                opMode.telemetry.addData("x", poseEstimate.getY());
//                opMode.telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
//                opMode.telemetry.addData("front distance", rangeClaw);
//                opMode.telemetry.update();
            }
            if (IsDriverControl) {
                if(IsFieldCentric) driveControlsFieldCentric();
                if(!IsFieldCentric) driveControlsRobotCentric();
            }
        }
    }
}

