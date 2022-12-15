package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

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
@Autonomous(name = "AutoAsyncDriveWithStates")
public class AutoAsyncDriveWithStates extends LinearOpMode {
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
    enum DriveState {
        IDLE,
        DriveToFirstJunction,
        DriveToConeStackFromFirstJunction,
        DriveToJunction2FromConeStack,
        DriveToConeStackFromJunction2
        // TODO: Create remaining Drive to other juntions states
    }
    enum ArmState {
        ArmAtHome,
        ArmRaiseSleep1,
        ArmRaiseSlideWait1,
        ArmRaiseSlideWait2,
        ArmRaiseSleep2,
        ArmRaisedHighPosition,
        ArmLowerSleep1,
        ArmLowerSlideWait1,
        ArmLowerSlideWait2,
        ArmLowerSleep2,
        ArmReadyToPickupFromStack
    }
    private ElapsedTime ArmRaiseSleep1ElapsedTime = new ElapsedTime();
    private ElapsedTime ArmRaiseSleep2ElapsedTime = new ElapsedTime();
    private ElapsedTime ArmLowerSleep1ElapsedTime = new ElapsedTime();
    private ElapsedTime ArmLowerSleep2ElapsedTime = new ElapsedTime();
    DriveState driveState = DriveState.IDLE;
    ArmState armState = ArmState.ArmAtHome;

    @Override
    public void runOpMode() throws InterruptedException {
        AprilTags();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        InitServosMotors();

        // Starting position of robot on field
        // TODO: Review and correct
        Pose2d startPose = new Pose2d(-36, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(45, -20), Math.toRadians(90))
                .build();

        waitForStart();
        // Set runtime to zero and start counting.
        runtime.reset();
        if (isStopRequested()) return;
        // Set the current state to TRAJECTORY_1, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        driveState = DriveState.DriveToFirstJunction;
        drive.followTrajectoryAsync(trajectory1);
        armState = ArmState.ArmRaiseSleep1;
        ArmRaiseSleep1ElapsedTime.reset();
        // TODO: Do actions before first arm sleep

        while (opModeIsActive() && !isStopRequested() && runtime.seconds() < 30) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            // We essentially define the flow of the state machine through this switch statement
            switch (driveState) {
                case DriveToFirstJunction:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    EvaluateAndAdjustArmState();
                    if (!drive.isBusy() && armState == ArmState.ArmRaisedHighPosition) {
                        // TODO: Drop Cone
                        driveState = DriveState.DriveToConeStackFromFirstJunction;
                        Trajectory trajectoryToConeStack = drive.trajectoryBuilder(startPose)
                                .splineTo(new Vector2d(45, -20), Math.toRadians(90))
                                .build();
                        drive.followTrajectoryAsync(trajectoryToConeStack);
                        ArmLowerSleep1ElapsedTime.reset();
                        armState = ArmState.ArmLowerSleep1;
                    }
                    break;
                case DriveToConeStackFromFirstJunction:
                    EvaluateAndAdjustArmState();
                    if (!drive.isBusy() && armState == ArmState.ArmReadyToPickupFromStack) {
                        // TODO: Pickup cone from stack
                        driveState = DriveState.DriveToJunction2FromConeStack;
                        Trajectory trajectoryToJunction1FromStack = drive.trajectoryBuilder(startPose)
                                .splineTo(new Vector2d(45, -20), Math.toRadians(90))
                                .build();
                        drive.followTrajectoryAsync(trajectoryToJunction1FromStack);
                        ArmRaiseSleep1ElapsedTime.reset();
                        armState = ArmState.ArmRaiseSleep1;
                    }
                    break;
                case DriveToJunction2FromConeStack:
                    EvaluateAndAdjustArmState();
                    if (!drive.isBusy() && armState == ArmState.ArmRaisedHighPosition) {
                        // TODO: Drop Cone
                        driveState = DriveState.DriveToConeStackFromJunction2;
                        Trajectory trajectoryToConeStackFromJunction2 = drive.trajectoryBuilder(startPose)
                                .splineTo(new Vector2d(45, -20), Math.toRadians(90))
                                .build();
                        drive.followTrajectoryAsync(trajectoryToConeStackFromJunction2);
                        ArmLowerSleep1ElapsedTime.reset();
                        armState = ArmState.ArmLowerSleep1;
                    }
                    break;

                    // TODO: Handle all remaining cases for trajectories to/from junctions to cone stack
            }

            // Anything outside of the switch statement will run independent of the currentState

            // We update drive continuously in the background, regardless of state
            drive.update();
            // We update our lift PID continuously in the background, regardless of state
            //lift.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Continually write pose to `PoseStorage`
            //PoseStorage.currentPose = poseEstimate;

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }

    private void EvaluateAndAdjustArmState() {
        switch (armState) {
            case ArmRaiseSleep1:
                // TODO: Sleep the right amount here.
                if (ArmRaiseSleep1ElapsedTime.milliseconds() >= 200) {
                    // TODO: Do arm actions after sleep
                    armState = ArmState.ArmRaiseSlideWait1;
                }
                break;
            case ArmRaiseSlideWait1:
                if (!slideOne.isBusy() && !slideTwo.isBusy()) {
                    // TODO: Do next step of arm actions
                    // Including raising arm to next position
                    armState = ArmState.ArmRaiseSlideWait2;
                }
                break;
            case ArmRaiseSlideWait2:
                if (!slideOne.isBusy() && !slideTwo.isBusy()) {
                    // TODO: Do next step of arm actions
                    ArmRaiseSleep2ElapsedTime.reset();
                    armState = ArmState.ArmRaiseSleep2;
                }
                break;
            case ArmRaiseSleep2:
                // TODO: Sleep the right amount here.
                if (ArmRaiseSleep1ElapsedTime.milliseconds() >= 1000) {
                    // TODO: Do arm actions after sleep
                    armState = ArmState.ArmRaisedHighPosition;
                }
                break;
            case ArmLowerSleep1:
                // TODO: Sleep the right amount here.
                if (ArmLowerSleep1ElapsedTime.milliseconds() >= 200) {
                    // TODO: Do arm actions after sleep
                    armState = ArmState.ArmLowerSlideWait1;
                }
                break;
            case ArmLowerSlideWait1:
                if (!slideOne.isBusy() && !slideTwo.isBusy()) {
                    // TODO: Do next step of arm actions
                    // Including lowering arm to next position
                    armState = ArmState.ArmLowerSlideWait2;
                }
                break;
            case ArmLowerSlideWait2:
                if (!slideOne.isBusy() && !slideTwo.isBusy()) {
                    // TODO: Do next step of arm actions
                    ArmLowerSleep2ElapsedTime.reset();
                    armState = ArmState.ArmLowerSleep2;
                }
                break;
            case ArmLowerSleep2:
                // TODO: Sleep the right amount here.
                if (ArmLowerSleep1ElapsedTime.milliseconds() >= 1000) {
                    // TODO: Do arm actions after sleep
                    armState = ArmState.ArmReadyToPickupFromStack;
                }
                break;
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
        slideOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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