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
@Autonomous(name = "AutoRouteTest with Markers")
public class markerTest_no_while_isbusy extends LinearOpMode {
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
    Servo spinTwo;
    Servo armRote;
    Servo liftWrist;
    Servo armGrip;
    DcMotor slideOne;
    DcMotor slideTwo;
    DistanceSensor rearDistance;
    DistanceSensor clawDistance;
    DistanceSensor frontDistance;
    double  position1 = 0.95; // spinOne
    double  position2 = 0.95; // spinTwo
    double  position3 = 0.13; // armRote
    double  position4 = 0.6;  // liftWrist
    double  position5 = 0.0;  // armGrip
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        AprilTags();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        InitServosMotors();
        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL);

        // Starting position of robot on field
        Pose2d startPose = new Pose2d(-36, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        if (opModeIsActive()) {
            if (tagOfInterest.id==11){
                //to first spot
                TrajectorySequence Park1 = drive.trajectorySequenceBuilder(startPose)
                        .forward(26)
                        .strafeLeft(24)
                        .build();
                drive.followTrajectorySequence(Park1);
            }
            else if (tagOfInterest.id==14){
                //to second spot
                TrajectorySequence Start2 = drive.trajectorySequenceBuilder(startPose)
                        // claw is closed as part of init
                        // added "run to position" mode in init code
                        // marker tells slides to lift cone (no while or isbusy) at time 0
                        .addTemporalMarker(0, () -> {
                            position5 = 0;                      // armGrip position
                            armGrip.setPosition(position5);     // make sure claw is closed
                            slideOne.setTargetPosition(1400);   // set slide height
                            slideTwo.setTargetPosition(1400);   // set slide height
                            slideOne.setPower(1);               // raise slide elevator
                            slideTwo.setPower(1);               // raise slide elevator
                        })

                        // since there is no isbusy loop - need to allow time for slide to extend before swinging arm
                        // estimating slide extension will take 1.5 seconds - this is the start time of the next marker
                        .addTemporalMarker(1.5, () -> {
                            slideOne.setPower(0);               // set slide motor power to zero (don't know if this is really needed or not)
                            slideTwo.setPower(0);               // set slide motor power to zero (don't know if this is really needed or not)
                            position1 = .80;                    // spinOne position
                            position2 = .80;                    // spinTwo position
                            spinOne.setPosition(position1);     // swing arm to stow position
                            spinTwo.setPosition(position2);     // swing arm to stow position
                        })

                        // need to allow time for the swing arm to stow cone before next action ~0.15 seconds
                        .addTemporalMarker(1.65, () -> {
                            position5 = 0;                      // armGrip position
                            armGrip.setPosition(position5);     // make sure claw is still closed
                            slideOne.setTargetPosition(1738);   // set slide height
                            slideTwo.setTargetPosition(1738);   // set slide height
                            slideOne.setPower(1);               // raise slide elevator
                            slideTwo.setPower(1);               // raise slide elevator
                            position1 = .11;                    // spinOne position
                            position2 = .11;                    // spinTwo position
                            spinOne.setPosition(position1);     // swing arm to high position
                            spinTwo.setPosition(position2);     // swing arm to high position
                        })

                          // need to allow time for the swing arm to move cone to high position before next action ~1 seconds
                        .addTemporalMarker(2.65, () -> {
                            slideOne.setPower(0);               // set slide motor power to zero (don't know if this is really needed or not)
                            slideTwo.setPower(0);               // set slide motor power to zero (don't know if this is really needed or not)
                            position3 = .83;                    // armRote position
                            position4 = .13;                    // liftWrist position
                            armRote.setPosition(position3);     // rotate arm 180 degrees
                            liftWrist.setPosition(position4);   // set wrist position for cone delivery
                        })

                        // move forward 12 inches
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
                                new Pose2d(-36, -12, Math.toRadians(-135)), Math.toRadians(45),
                                SampleMecanumDrive.getVelocityConstraint(15,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(15))
                        // drive LG at slower speed toward junction pole
                        .splineToSplineHeading(new Pose2d(-31, -3, Math.toRadians(-135)), Math.toRadians(45),
                                SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(15))
                        .build();

                drive.followTrajectorySequence(Start2);

                //        .forward(12)
                //        .turn(Math.toRadians(180))
                //        .build();
                //while(drive.isBusy()){}
                //position5 = 0;
                //position1 = .80;
                //position2 = .80;
                // armGrip.setPosition(position5);
                //for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(200); stop>System.nanoTime();) {}
                //slideOne.setTargetPosition(1400);
                //slideTwo.setTargetPosition(1400);
                //slideOne.setPower(1);
                //slideTwo.setPower(1);
                //slideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //slideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //while ((slideOne.isBusy()) && (slideTwo.isBusy())){}
                //slideOne.setPower(0);
                //slideTwo.setPower(0);
                //spinOne.setPosition(position1);
                //spinTwo.setPosition(position2);
                //position5 = 0;
                //position1 = .11;
                //position2 = .11;
                //position3 = .83;
                //position4 = .13;
                //armGrip.setPosition(position5);
                //slideOne.setTargetPosition(1738);
                //slideTwo.setTargetPosition(1738);
                //slideOne.setPower(1);
                //slideTwo.setPower(1);
                //slideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //slideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //while ((slideOne.isBusy()) && (slideTwo.isBusy())){}
                //slideOne.setPower(0);
                //slideTwo.setPower(0);
                //spinOne.setPosition(position1);
                //spinTwo.setPosition(position2);
                //for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(1000); stop>System.nanoTime();) {}
                //armRote.setPosition(position3);
                //liftWrist.setPosition(position4);
                //startPose = new Pose2d(-36,-48, Math.toRadians(-90));
                //TrajectorySequence Spin2 = drive.trajectorySequenceBuilder(startPose)
                //        .setReversed(true)
                //        .splineToSplineHeading(new Pose2d(-30, -6, Math.toRadians(-135)), Math.toRadians(45))
                //        .build();
                //drive.followTrajectorySequence(Spin2);
            }
            else if(tagOfInterest.id==19) {
                //to third spot
                TrajectorySequence Park3 = drive.trajectorySequenceBuilder(startPose)
                        .forward(26)
                        .strafeRight(24)
                        .build();
                drive.followTrajectorySequence(Park3);
            }
            else{
                //if it don't read
                TrajectorySequence Park2 = drive.trajectorySequenceBuilder(startPose)
                        .forward(26)
                        .build();
                drive.followTrajectorySequence(Park2);
            }
        }
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
        slideOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);      // added to use encoder
        slideTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);      // added to use encoder
        slideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);        // added to run to position
        slideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);        // added to run to position


        armGrip.setPosition(position5);     // close claw on initialization
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