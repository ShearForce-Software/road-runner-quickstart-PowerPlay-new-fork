package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

    // Define Motor, Servo, and Sensor objects  (Make them private so they can't be accessed externally)
    private DcMotor leftFront               = null;
    private DcMotor leftRear                = null;
    private DcMotor rightFront              = null;
    private DcMotor rightRear               = null;
    private DcMotor slideOne                = null;
    private DcMotor slideTwo                = null;
    private Servo  spinOne                  = null;
    private Servo  spinTwo                  = null;
    private Servo  armRote                  = null;
    private Servo  liftWrist                = null;
    private Servo  armGrip                  = null;
    private DistanceSensor rearDistance     = null;
    private DistanceSensor clawDistance     = null;
    private DistanceSensor frontDistance    = null;
    private BNO055IMU imu                   = null;
    private static final double SLIDE_POWER = 1;

    // Define Servo start positions
    private double  spinOne_pos             = 0.95;
    private double  spinTwo_pos             = 0.95;
    private double  armRote_pos             = 0.11;
    private double  liftWrist_pos           = 0.6;
    private double  armGrip_pos             = 0.0;

    // Define Slide Encoder Positions
    int START_POS                           = 5;
    int STOW_POS                            = 1400;
    int LOW_POS                             = 1850;   //2090
    int MED_POS                             = 3560;   //3681
    int HIGH_POS                            = 1550;  //1738

    boolean high = false;
    boolean stow = false;
    boolean ready = false;
    boolean intake = true;


    public void Init (HardwareMap hardwareMap) {
        // Define and Initialize Chassis Motors
        leftFront       = hardwareMap.get(DcMotor.class, "leftFront");
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);

        leftRear        = hardwareMap.get(DcMotor.class, "leftRear");
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);

        rightFront      = hardwareMap.get(DcMotor.class, "rightFront");
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        rightRear       = hardwareMap.get(DcMotor.class, "rightRear");
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // Define and Initialize Slide Motors
        slideOne        = hardwareMap.get(DcMotor.class, "slideOne");
        slideOne.setDirection(DcMotor.Direction.FORWARD);
        slideOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideTwo        = hardwareMap.get(DcMotor.class, "slideTwo");
        slideTwo.setDirection(DcMotor.Direction.FORWARD);
        slideTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Define and Initialize Servos to the start position
        spinOne         = hardwareMap.get(Servo.class, "spinOne");
        spinOne.setDirection(Servo.Direction.FORWARD);
        spinOne.setPosition(spinOne_pos);

        spinTwo         = hardwareMap.get(Servo.class, "spinTwo");
        spinTwo.setDirection(Servo.Direction.REVERSE);
        spinTwo.setPosition(spinTwo_pos);

        armRote         = hardwareMap.get(Servo.class, "armRote");
        armRote.setDirection(Servo.Direction.FORWARD);
        armRote.setPosition(armRote_pos);

        liftWrist       = hardwareMap.get(Servo.class, "liftWrist");
        liftWrist.setDirection(Servo.Direction.FORWARD);
        liftWrist.setPosition(liftWrist_pos);

        armGrip         = hardwareMap.get(Servo.class, "armGrip");
        armGrip.setDirection(Servo.Direction.FORWARD);
        armGrip.setPosition(armGrip_pos);

        // Define Distance Sensors
        rearDistance    = hardwareMap.get(DistanceSensor.class, "rearDistance");
        clawDistance    = hardwareMap.get(DistanceSensor.class, "clawDistance");
        frontDistance   = hardwareMap.get(DistanceSensor.class, "frontDistance");

        // IMU hardware for field centric control
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

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

    public void GrabFromStack(Drive drive) {}

    public void GoToHigh() {}

    public void GoToMedium() {}

    public void GoToLow() {}

    public void ReturnFromHigh() {}

    public void ReturnFromLowMedium() {}

    public void Stow() {}

    private void WaitForTrajectoryToFinish(SampleMecanumDrive drive) {
        while(drive.isBusy()) {
            drive.update();
        }
    }

    private void WaitForSlides(SampleMecanumDrive drive) {
        while ((slideOne.isBusy()) || (slideTwo.isBusy())) {
            drive.update();
        }
    }

    private void SpecialSleep(SampleMecanumDrive drive, long milliseconds) {
        for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(milliseconds); stop>System.nanoTime();) {
            drive.update();
        }
    }
}

