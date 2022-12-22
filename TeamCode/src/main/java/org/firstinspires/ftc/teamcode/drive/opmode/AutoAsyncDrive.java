package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

/*
 * Op mode to experiment with Road Runner Autonomous Routes.
 * Utilization of the dashboard is recommended to visualize routes and check path following.
 * To access the dashboard, connect your computer to the RC's WiFi network.
 * In your browser, navigate to https://192.168.49.1:8080/dash if you're using the RC phone
 * or https://192.168.43.1:8080/dash if you are using the Control Hub.
 * Once successfully connected, start the program, and Li'l Gerry will begin driving the route.
 * You should observe the target position (green) and your pose estimate (blue) and adjust your
 * follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 */
@Config
@Autonomous(name = "AutoAsyncDrive")
public class AutoAsyncDrive extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    AprilTagDetection tagOfInterest = null;

    Servo spinOne;
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
    private ElapsedTime runtime = new ElapsedTime();

    int START_POS = 5;
    int STOW_POS = 1400;
    int LOW_POS = 1850;   //2090
    int MED_POS = 3560;   //3681
    int HIGH_POS = 1550;  //1738
    boolean high = false;
    boolean movingDown = false;
    boolean stow = false;
    boolean ready = false;
    public static final double ARM_POWER    =  1 ;

    @Override
    public void runOpMode() throws InterruptedException {
        AprilTags();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        InitServosMotors();

        // Starting position of robot on field
        Pose2d startPose = new Pose2d(-36, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        waitForStart();
        // Set runtime to zero and start counting.
        runtime.reset();
        if (isStopRequested()) return;
        if (opModeIsActive()) {
            // TODO:  Review this trajectory, chain together the correct path to make it all
            // the way to the first junction.
            TrajectorySequence Start2 = drive.trajectorySequenceBuilder(startPose)
                    //.setConstraints(10, 10, Math.toRadians(180), Math.toRadians(180), 12)
                    .forward(12,
                            SampleMecanumDrive.getVelocityConstraint(15,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(15))
                    // rotate 180 degree for cone delivery
                    .splineToSplineHeading(
                            new Pose2d(-36, -24, Math.toRadians(-90)), Math.toRadians(90),
                            SampleMecanumDrive.getVelocityConstraint(15,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(15))
                    // position LG for junction pole delivery (fast)
                    .splineToSplineHeading(
                            new Pose2d(-36, -12, Math.toRadians(-135)), Math.toRadians(90),
                            SampleMecanumDrive.getVelocityConstraint(15,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(15))
                    // drive LG at slower speed toward junction pole
                    .splineToSplineHeading(new Pose2d(-30, -2, Math.toRadians(-135)), Math.toRadians(45),
                            SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(15))
                    .waitSeconds(.5)
                    .splineToSplineHeading(new Pose2d(-36, -24, Math.toRadians(-90)), Math.toRadians(-90),
                            SampleMecanumDrive.getVelocityConstraint(15,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(15))
                    .forward(8,
                            SampleMecanumDrive.getVelocityConstraint(15,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(15))
                    .build();
            drive.followTrajectorySequenceAsync(Start2);

            // Keep in mind portions of code that do the same thing can be refactored to methods.
            // or refactor to methods just for readability.
            // armGrip close position
            armGrip.setPosition(0); // close claw
            if (!WaitForServo(drive, armGrip, 0)) return;
            //************************************************************
            // raise slide to cone stow position height
            //************************************************************
            telemetry.addData("slide target pos before",slideOne.getTargetPosition());
            slideOne.setTargetPosition(STOW_POS);
            slideTwo.setTargetPosition(STOW_POS);
            slideOne.setPower(ARM_POWER);
            slideTwo.setPower(ARM_POWER);
            telemetry.addData("slide target pos after",slideOne.getTargetPosition());
            if (!WaitForSlides(drive)) return;
            slideOne.setPower(0);
            slideTwo.setPower(0);
            //************************************************************
            // straighten wrist before rotating 180 degrees
            //************************************************************
            // liftWrist straight position
            liftWrist.setPosition(.35);   // straighten wrist
            if (!WaitForServo(drive, liftWrist, .35)) return;
            //slideHeight();}
            //************************************************************
            // rotate arm 180 degrees to flip cone
            //***********`*************************************************
            // armRote position
            armRote.setPosition(.82); // rotate arm 180 degrees
            if (!WaitForServo(drive, armRote, .82)) return;
            //slideHeight();}
            //************************************************************
            // spin arm to cone stow rotate position
            //*************************************
            // position1 = .84; //.72;                // spinOne cone stow rotate position
            // spinTwo cone stow rotate position
            spinOne.setPosition(.84); // spin arm to cone stow rotate position
            spinTwo.setPosition(.84); // spin arm to cone stow rotate position
//
//            if (!SpecialSleep(drive, 0)) return;
//            // TODO: Set arm grip
//            if (!SpecialSleep(drive, 200)) return;
//            // TODO: Start slides
//            if (!WaitForSlides(drive)) return;
//            // TODO: Do arm movements, then wait for slide again?
//            if (!WaitForSlides(drive)) return;
//            // TODO: More arm movements
//            if (!SpecialSleep(drive, 1000)) return;
//            // TODO: More arm and wrist action
//            if (!WaitForTrajectoryToFinish(drive)) return;
//
//            // TODO: Seek for junction (if necessary) and Drop code
//
//            // TODO: Start next trajectory async to stack of cones.
//            //drive.followTrajectorySequenceAsync(blah);
//            // TODO: Move arm while driving
//            if (!WaitForTrajectoryToFinish(drive)) return;
//
//            // TODO: Seek and pickup cone
//
//            // TODO: Start next trajectory to next junction and move arm while driving.
            // Once picked up, you can call a method you create that stows and/or raises to the
            // junction position you want.
            // Make sure to use our special wait and sleep methods.
        }
    }

    private boolean WaitForTrajectoryToFinish(SampleMecanumDrive drive) {
        while(drive.isBusy()) {
            if (ShouldStop()) return false;
            drive.update();
        }
        return true;
    }

    private boolean
    WaitForSlides(SampleMecanumDrive drive) {
        while ((slideOne.isBusy()) || (slideTwo.isBusy())) {
            if (ShouldStop()) return false;
            drive.update();
        }
        return true;
    }

    private boolean WaitForServo(SampleMecanumDrive drive, Servo servo, double target) {
//        for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(milliseconds); stop>System.nanoTime();) {
//            if (ShouldStop()) return false;
//            drive.update();
//        }
          while(servo.getController().getServoPosition(servo.getPortNumber())!=target){
              if (ShouldStop()) return false;
              drive.update();
          }
        return true;
    }

    private boolean ShouldStop() {
        // Should we stop at 29 seconds to account for human delay pressing start?
        return !opModeIsActive() || isStopRequested() || runtime.seconds() >= 30;
    }

    private void InitServosMotors() {
        spinOne = hardwareMap.get(Servo.class, "spinOne");
        spinTwo = hardwareMap.get(Servo.class, "spinTwo");
        armRote = hardwareMap.get(Servo.class, "armRote");
        liftWrist = hardwareMap.get(Servo.class, "liftWrist");
        armGrip = hardwareMap.get(Servo.class, "armGrip");
        slideOne = hardwareMap.get(DcMotor.class, "slideOne");
        slideTwo = hardwareMap.get(DcMotor.class, "slideTwo");

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

        armGrip.setPosition(position5);
        sleep(2000);
        spinOne.setPosition(position1);
        spinTwo.setPosition(position2);
        armRote.setPosition(position3);
        liftWrist.setPosition(position4);
    }

    private void AprilTags() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        telemetry.setMsTransmissionInterval(50);
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == 11 || tag.id == 14 || tag.id == 19)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest == null){
            /*
             * Insert your autonomous code here, presumably running some default configuration
             * since the tag was never sighted during INIT
             */
        }
        else{
            /*
             * Insert your autonomous code here, probably using the tag pose to decide your configuration.
             */

            // e.g.
        }
    }

    void tagToTelemetry(AprilTagDetection detection){
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}