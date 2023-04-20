package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
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
    DistanceSensor rightDistance;
    DistanceSensor clawDistance;
    DistanceSensor leftDistance;
    ColorSensor sensorColorLeft;
    ColorSensor sensorColorRight;
    BNO055IMU imu;

    double rangeRight;
    double rangeClaw;
    double rangeLeft;

    int START_POS = 10; //5
    int STOW_POS = 1080; //1400
    int LOW_POS = 1250;   //1850
    int MED_POS = 2370;   //3560
    int HIGH_POS = 1510;//550     900
    int STACK_POS = 550; //1100

    //FindCondeCenter variables
    public double forwardLG, shiftLG;
    private double finalLeft, finalRight, rawRangeLeft, rawRangeRight,redConeDetect, blueConeDetect;
    double RedLeftSensorOffset = 0.49;
    double RedRightSensorOffset = 0.31;
    double BlueLeftSensorOffset = 0.39;
    double BlueRightSensorOffset = 0.28;

    double shiftRedScaleValue = 0.925;
    double forward_both_RedSensorScaleValue = -0.15;
    double forward_single_RedSensorScaleValue1 = -0.45;
    double forward_single_RedSensorScaleValue2 = -0.4;

    double shiftBlueScaleValue = 0.95;
    double forward_both_BlueSensorScaleValue = -0.1;
    double forward_single_BlueSensorScaleValue1 = -0.3;
    double forward_single_BlueSensorScaleValue2 = -0.2;

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
        rightDistance = hardwareMap.get(DistanceSensor.class, "rearDistance");
        clawDistance = hardwareMap.get(DistanceSensor.class, "clawDistance");
        leftDistance = hardwareMap.get(DistanceSensor.class, "frontDistance");
        sensorColorLeft = hardwareMap.get(ColorSensor.class, "frontDistance");
        sensorColorRight = hardwareMap.get(ColorSensor.class, "rearDistance");
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

    public void StartPosition(SampleMecanumDrive drive, boolean isDriverControl) {
        if(isDriverControl) {
            slideOne.setTargetPosition(0);
            slideTwo.setTargetPosition(0);
            slideOne.setPower(ARM_POWER);
            slideTwo.setPower(ARM_POWER);
            WaitForSlides(drive);
            slideOne.setPower(0);
            slideTwo.setPower(0);
            armGrip.setPosition(.145);
            SpecialSleep(null, 180);
            spinOne.setPosition(1);
            spinTwo.setPosition(1);
            armRote.setPosition(.11);
            liftWrist.setPosition(.53);
        }
        else{
            slideOne.setTargetPosition(0);
            slideTwo.setTargetPosition(0);
            slideOne.setPower(ARM_POWER);
            slideTwo.setPower(ARM_POWER);
            WaitForSlides(drive);
            slideOne.setPower(0);
            slideTwo.setPower(0);
            spinOne.setPosition(0.79);
            spinTwo.setPosition(0.79);
            armRote.setPosition(0.83);
            liftWrist.setPosition(0.63);
            armGrip.setPosition(0);
            SpecialSleep(null, 180);
            slideOne.setTargetPosition(115);
            slideTwo.setTargetPosition(115);
            slideOne.setPower(ARM_POWER);
            slideTwo.setPower(ARM_POWER);
            WaitForSlides(drive);
            spinOne.setPosition(0.81);
            spinTwo.setPosition(0.81);
            liftWrist.setPosition(0.88);
        }
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
        spinOne.setPosition(.18);
        spinTwo.setPosition(.18);
        liftWrist.setPosition(.02);
        //SpecialSleep(drive, 180);
        //************************************************************
        // raise slides to high junction delivery height
        //************************************************************
        slideOne.setTargetPosition(1610);
        slideTwo.setTargetPosition(1610);
        //OLD ARM POSITIONS PRE DANGLY-BIT >>
//        spinOne.setPosition(.14);
//        spinTwo.setPosition(.14);
//        liftWrist.setPosition(.13);
//        //SpecialSleep(drive, 180);
//        //************************************************************
//        // raise slides to high junction delivery height
//        //************************************************************
//        slideOne.setTargetPosition(HIGH_POS);
//        slideTwo.setTargetPosition(HIGH_POS);
        slideOne.setPower(ARM_POWER);
        slideTwo.setPower(ARM_POWER);
        WaitForSlides(drive);
        slideOne.setPower(0);
        slideTwo.setPower(0);
        //************************************************************
        // wrist to delivery angle (high)
        //************************************************************

        // set stow variable to false
        stow = false;
        // set ready for delivery variable to true
        readyToDrop = true;
    }
    public void GoToHigh180(SampleMecanumDrive drive){
        armGrip.setPosition(0);
        spinOne.setPosition(.18);
        spinTwo.setPosition(.18);
        liftWrist.setPosition(.02);
        slideOne.setTargetPosition(1610);
        slideTwo.setTargetPosition(1610);
        slideOne.setPower(ARM_POWER);
        slideTwo.setPower(ARM_POWER);
        slideOne.setPower(ARM_POWER);
        slideTwo.setPower(ARM_POWER);
        WaitForSlides(drive);
        slideOne.setPower(0);
        slideTwo.setPower(0);

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
        //WaitForSlides(drive);
        //slideOne.setPower(0);
        //slideTwo.setPower(0);
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
//        WaitForSlides(drive);
//        slideOne.setPower(0);
//        slideTwo.setPower(0);
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
        SpecialSleep(drive, 60);
        //************************************************************
        // Close claw
        //************************************************************
        armGrip.setPosition(0);
        SpecialSleep(drive, 120);
        //************************************************************
        // spin arm to safe return position
        //************************************************************
        spinOne.setPosition(1);
        spinTwo.setPosition(1);
        SpecialSleep(drive, 180);
        //************************************************************
        // straighten wrist
        //************************************************************
        liftWrist.setPosition(0.35);
        SpecialSleep(drive, 180);
        //************************************************************
        // rotate arm 180 degrees
        //************************************************************
        armRote.setPosition(0.11);
        SpecialSleep(drive,500);
        //************************************************************
        // wrist to cone pickup position
        //************************************************************
        liftWrist.setPosition(0.53);
        SpecialSleep(drive, 120);
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
        SpecialSleep(drive, 60);
        //************************************************************
        // Close claw
        //************************************************************
        armGrip.setPosition(0);
        liftWrist.setPosition(1);
        //************************************************************
        // rotate arm 180 degrees (so gripper is backwards)
        //************************************************************
        armRote.setPosition(0.11);
        //************************************************************
        // spin arm to cone pickup position
        //************************************************************
        SpecialSleep(drive, 500);
        spinOne.setPosition(1);
        spinTwo.setPosition(1);
        //************************************************************
        // wrist to cone pickup position
        //************************************************************
        //liftWrist.setPosition(0.57);
        SpecialSleep(drive, 360);
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
        liftWrist.setPosition(.53);
        armGrip.setPosition(0.145);
        intake = true;
    }

    public void ReadyToGrabFromStack(SampleMecanumDrive drive) {
        armGrip.setPosition(0); // Close claw
        armRote.setPosition(0.11); // rotate arm 180 degrees (so gripper is backwards)
        liftWrist.setPosition(1);
        SpecialSleep(drive, 500);
        spinOne.setPosition(.93); // spin arm to cone pickup position//1
        spinTwo.setPosition(.93);//1
        liftWrist.setPosition(0.6); // wrist to cone pickup position
        SpecialSleep(drive, 300);
        slideOne.setTargetPosition(STACK_POS);
        slideTwo.setTargetPosition(STACK_POS);
        slideOne.setPower(ARM_POWER);
        slideTwo.setPower(ARM_POWER);
        armRote.setPosition(.11);
        liftWrist.setPosition(.53);
        SpecialSleep(drive, 200);
        armGrip.setPosition(.14);
    }

    public void GrabFromStack(SampleMecanumDrive drive) {
        spinOne.setPosition(1);
        spinTwo.setPosition(1);
        SpecialSleep(drive, 200);
        armGrip.setPosition(0);
        SpecialSleep(drive, 200);
        slideOne.setPower(ARM_POWER);
        slideTwo.setPower(ARM_POWER);
        slideOne.setTargetPosition(1610);
        slideTwo.setTargetPosition(1610);
//        WaitForSlides(drive);
//        slideOne.setPower(0);
//        slideTwo.setPower(0);
    }
    private boolean slideAdjustOn = false;

    public void ManualSlideAdjust(boolean up){

        if(up){
            slideOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideOne.setPower(1);
            slideTwo.setPower(1);
        }
        else{
            slideOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideOne.setPower(-1);
            slideTwo.setPower(-1);
        }
        slideAdjustOn = true;

    }
    public void ManualSlideAdjust(){
        if(!(slideOne.isBusy() && slideTwo.isBusy()) || slideAdjustOn){
            slideOne.setPower(0);
            slideTwo.setPower(0);
            slideOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideAdjustOn = false;
        }
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
        SpecialSleep(drive, 200);
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
        SpecialSleep(drive, 120);
        //slideHeight();}
        //************************************************************
        // rotate arm 180 degrees to flip cone
        //************************************************************
        // armRote position
        armRote.setPosition(.81); // rotate arm 180 degrees
        // wait for arm to rotate
        SpecialSleep(drive, 450);
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

    public void autoArmToHigh(SampleMecanumDrive drive){
        armGrip.setPosition(0); // close claw
        // wait for claw to close
        SpecialSleep(drive, 200);
        //************************************************************
        // raise slide to cone stow position height
        //************************************************************
        slideOne.setTargetPosition(HIGH_POS);
        slideTwo.setTargetPosition(HIGH_POS);
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
        SpecialSleep(drive, 120);
        //slideHeight();}
        //************************************************************
        // rotate arm 180 degrees to flip cone
        //************************************************************
        // armRote position
        armRote.setPosition(.83); // rotate arm 180 degrees
        // wait for arm to rotate
        SpecialSleep(drive, 450);
        //slideHeight();}
        //************************************************************
        // spin arm to cone high junct position
        //************************************************************
        spinOne.setPosition(.18); // spin arm to cone stow rotate position
        spinTwo.setPosition(.18); // spin arm to cone stow rotate position
        //************************************************************
        // verify claw is closed
        //*************************armGrip.setPosition(0);***********************************
        liftWrist.setPosition(.02);
        //SpecialSleep(drive, 180);
    }

    public void closeClaw(){
        armGrip.setPosition(0);
    }

    public void openClaw(){
        armGrip.setPosition(.145);
    }

    public void driveControlsRobotCentric() {
        double y = opMode.gamepad2.left_stick_y;
        double x = -opMode.gamepad2.left_stick_x * 1.1;
        double rx = opMode.gamepad2.right_stick_x;

        rangeRight = rightDistance.getDistance(DistanceUnit.CM);
        rangeClaw = clawDistance.getDistance(DistanceUnit.CM);
        rangeLeft = leftDistance.getDistance(DistanceUnit.CM);

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

        rangeRight = rightDistance.getDistance(DistanceUnit.CM);
        rangeClaw = clawDistance.getDistance(DistanceUnit.CM);
        rangeLeft = leftDistance.getDistance(DistanceUnit.CM);

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
    public void FindConeCenter(){
        rawRangeLeft = leftDistance.getDistance(DistanceUnit.INCH);
        rawRangeRight = rightDistance.getDistance(DistanceUnit.INCH);
        redConeDetect = (sensorColorLeft.red() + sensorColorRight.red()) / 2;
        blueConeDetect = (sensorColorLeft.blue() + sensorColorRight.blue()) / 2;
        //~~sort through data~~
        if (redConeDetect > blueConeDetect) {
            if (rawRangeLeft < 1.78) {
                rangeLeft = (rawRangeLeft * 1.352 - 0.089) - RedLeftSensorOffset;
            } else if (rawRangeLeft > 1.78) {
                rangeLeft = (rawRangeLeft * 3.333 - 3.777) - RedLeftSensorOffset;
            } else {
                rangeLeft = rawRangeLeft;
            }
            if (rawRangeRight < 2.23) {
                rangeRight = (rawRangeRight * 1.050 - 0.040) - RedRightSensorOffset;
            } else if (rawRangeRight > 2.23) {
                rangeRight = (rawRangeRight * 2.038 - 2.399) - RedRightSensorOffset;
            } else {
                rangeRight = rawRangeRight;
            }

            //apply left and right corrections
            shiftLG = (rangeRight - rangeLeft) * shiftRedScaleValue;

            //apply forward correction based on single sensor or both sensors seeing cone
            if (Math.abs((rangeRight - rangeLeft)) > 1 && Math.abs((rangeRight - rangeLeft)) < 2.1) {
                if (rangeLeft < rangeRight) {
                    forwardLG = rangeLeft - forward_single_RedSensorScaleValue1;
                } else {
                    forwardLG = rangeRight - forward_single_RedSensorScaleValue1;
                }
            } else if (Math.abs((rangeRight - rangeLeft)) > 2.1) {
                if (rangeLeft < rangeRight) {
                    forwardLG = rangeLeft - forward_single_RedSensorScaleValue2;
                } else {
                    forwardLG = rangeRight - forward_single_RedSensorScaleValue2;
                }
            } else {
                forwardLG = ((rangeRight + rangeLeft) / 2) - forward_both_RedSensorScaleValue;
            }
        }

        // apply blue range calibration equations
        if (blueConeDetect > redConeDetect) {
            if (rawRangeLeft < 2.0) {
                rangeLeft = (rawRangeLeft * 1.290 - 0.292) - BlueLeftSensorOffset;
            } else if (rawRangeLeft > 2.0) {
                rangeLeft = (rawRangeLeft * 3.947 - 5.623) - BlueLeftSensorOffset;
            } else {
                rangeLeft = rawRangeLeft;
            }
            if (rawRangeRight < 2.55) {
                rangeRight = (rawRangeRight * 0.948 - 0.085) - BlueRightSensorOffset;
            } else if (rawRangeRight > 2.55) {
                rangeRight = (rawRangeRight * 2.838 - 5.360) - BlueRightSensorOffset;
            } else {
                rangeRight = rawRangeRight;
            }
            shiftLG = (rangeRight - rangeLeft) * shiftBlueScaleValue;

            //apply forward correction based on single sensor or both sensors seeing cone
            if (Math.abs((rangeRight - rangeLeft)) > 1 && Math.abs((rangeRight - rangeLeft)) < 2.1) {
                if (rangeLeft < rangeRight) {
                    forwardLG = rangeLeft - forward_single_BlueSensorScaleValue1;
                } else {
                    forwardLG = rangeRight - forward_single_BlueSensorScaleValue1;
                }
            } else if (Math.abs((rangeRight - rangeLeft)) > 2.1) {
                if (rangeLeft < rangeRight) {
                    forwardLG = rangeLeft - forward_single_BlueSensorScaleValue2;
                } else {
                    forwardLG = rangeRight - forward_single_BlueSensorScaleValue2;
                }
            } else {
                forwardLG = ((rangeRight + rangeLeft) / 2) - forward_both_BlueSensorScaleValue;
            }
        }

        /*
        if(((sensorColorLeft.red() + sensorColorRight.red()) / 2) > ((sensorColorLeft.blue() + sensorColorRight.blue()) / 2)){
            //red values
            if(rawRangeLeft < 1.78){
                finalLeft = (rawRangeLeft * 1.352 - 0.089) - 0.15;
            }
            else if(rawRangeLeft > 1.78){
                finalLeft = (rawRangeLeft * 3.333 - 3.777) - 0.15;
            }
            else{
                finalLeft = rawRangeLeft;
            }
            if(rawRangeRight < 2.23){
                finalRight = (rawRangeRight * 1.050 - 0.040) - 0.15;
            }
            else if(rawRangeRight > 2.23){
                finalRight = (rawRangeRight * 2.038 - 2.399) - 0.15;
            }
            else{
                finalRight = rawRangeRight;
            }
            //find shift and forward values
            shiftLG = (finalRight - finalLeft) * .85;
            forwardLG = ((finalRight + finalLeft)/2) - .1;
            //if the math ever pushes over an inch
            if (shiftLG > 1.1) {
                shiftLG = 1.25;
            }
            else if (shiftLG < -1.1){
                shiftLG = -1.25;
            }
            if (shiftLG > 0.5) {
                forwardLG = finalLeft + 0.2;
            }
            else if (shiftLG < -0.5){
                forwardLG = finalRight + 0.2;
            }
        }
        else{
            //blue values
            if(rawRangeLeft<2.0){
                finalLeft = (rawRangeLeft * 1.290 - 0.292) - 0.1;
            }
            else if(rawRangeLeft>2.0){
                finalLeft = (rawRangeLeft * 3.947 - 5.623) - 0.1;
            }
            else{
                finalLeft = rawRangeLeft;
            }
            if(rawRangeRight<2.55){
                finalRight = (rawRangeRight * 0.948 - 0.085) - 0.1;
            }
            else if(rawRangeRight>2.55){
                finalRight = (rawRangeRight * 2.838 - 5.360) - 0.1;
            }
            else{
                finalRight = rawRangeRight;
            }
            //find shift and forward values
            shiftLG = (finalRight - finalLeft) * .85;
            forwardLG = ((finalRight + finalLeft)/2) - 0.15;
            //if the math ever pushes over an inch
            if (shiftLG > 1.1) {
                shiftLG = 1.25;
            }
            else if (shiftLG < -1.1){
                shiftLG = -1.25;
            }
            if (shiftLG > 0.5) {
                forwardLG = finalLeft + 0.1;
            }
            else if (shiftLG < -0.5){
                forwardLG = finalRight + 0.1;
            }
        }

         */
    }
}

