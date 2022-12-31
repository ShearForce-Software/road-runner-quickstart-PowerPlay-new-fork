package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
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

    @Override
    public void runOpMode() throws InterruptedException {
        ArmControl armControl = new ArmControl(false, false, this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // Starting position of robot on field
        Pose2d startPose = new Pose2d(-36, -64.5, Math.toRadians(90));
        Pose2d junctionPos = new Pose2d(-29,-8, Math.toRadians(-135));
        Pose2d stackPos = new Pose2d(-60, -13.5, Math.toRadians(-180));
        Pose2d linePos = new Pose2d(-40, -13.5, Math.toRadians(-180));
        drive.setPoseEstimate(startPose);
        armControl.Init(hardwareMap);
        armControl.StartPosition();

        TrajectorySequence FirstCone = drive.trajectorySequenceBuilder(startPose)
                //.setConstraints(10, 10, Math.toRadians(180), Math.toRadians(180), 12)
                .forward(12,
                        SampleMecanumDrive.getVelocityConstraint(15,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(15))
                .splineToSplineHeading(new Pose2d(-36, -24, Math.toRadians(-90)), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(15,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(15))
                .splineToSplineHeading(junctionPos, Math.toRadians(45),
                        SampleMecanumDrive.getVelocityConstraint(15,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(15))
                .build();

        TrajectorySequence ToStack = drive.trajectorySequenceBuilder(junctionPos)
                .setReversed(false)
                .splineToSplineHeading(linePos, Math.toRadians(-180),
                        SampleMecanumDrive.getVelocityConstraint(15,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(15))
                // slow down this portion of the trajectory
                .splineToLinearHeading(stackPos, Math.toRadians(-180),
                        SampleMecanumDrive.getVelocityConstraint(15,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(15))
                .build();
        TrajectorySequence ToHighJunction = drive.trajectorySequenceBuilder(stackPos)
                .setReversed(true)
                .splineToSplineHeading(linePos, Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(15,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(15))
                .splineToSplineHeading(junctionPos, Math.toRadians(45),
                        SampleMecanumDrive.getVelocityConstraint(15,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(15))
                .build();

        AprilTags();
        waitForStart();
        if (isStopRequested()) return;
        if (opModeIsActive()) {
            // TODO:  Review this trajectory, chain together the correct path to make it all
            // the way to the first junction.
            drive.followTrajectorySequenceAsync(FirstCone);

            //************HACK FOR BUG IN ASYNC DRIVE****************//
            armControl.slideOne.setTargetPosition(armControl.STOW_POS);
            armControl.slideTwo.setTargetPosition(armControl.STOW_POS);
            armControl.slideOne.setPower(armControl.ARM_POWER);
            armControl.slideTwo.setPower(armControl.ARM_POWER);
            armControl.WaitForSlides(drive);
            armControl.slideOne.setPower(0);
            armControl.slideTwo.setPower(0);

            armControl.StowCone(drive);
            armControl.GoToHigh(drive);
            armControl.WaitForTrajectoryToFinish(drive);
            for (int i = 0; i <5; i++){
                armControl.openClaw();
                drive.followTrajectorySequenceAsync(ToStack);
                armControl.SpecialSleep(drive, 1000);
                armControl.ReturnFromHigh(drive);
                armControl.WaitForTrajectoryToFinish(drive);
                //grab from stack
                drive.followTrajectorySequenceAsync(ToHighJunction);
                armControl.SpecialSleep(drive, 500);
                armControl.StowCone(drive);
                armControl.GoToHigh(drive);
                armControl.WaitForTrajectoryToFinish(drive);
            }
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
//            // TODO: Seek for junction (if necessary) and Drop code
//            // TODO: Start next trajectory async to stack of cones.
//            //drive.followTrajectorySequenceAsync(blah);
//            // TODO: Move arm while driving
//            if (!WaitForTrajectoryToFinish(drive)) return;
//            // TODO: Seek and pickup cone
//            // TODO: Start next trajectory to next junction and move arm while driving.
            // Once picked up, you can call a method you create that stows and/or raises to the
            // junction position you want.
            // Make sure to use our special wait and sleep methods.
        }
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