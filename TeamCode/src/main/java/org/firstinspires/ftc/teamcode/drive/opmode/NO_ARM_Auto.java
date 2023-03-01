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
@Autonomous(name = "EMERGENCY NO ARM Auto (Scan and Park)")
public class NO_ARM_Auto extends LinearOpMode {
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

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        AprilTags();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Starting position of robot on field
        Pose2d startPose = new Pose2d(36, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        if (opModeIsActive()) {

            Pose2d parkPose = new Pose2d(36,-32,Math.toRadians(-90));

            if (tagOfInterest.id==11){
                //to first spot
                TrajectorySequence Park1 = drive.trajectorySequenceBuilder(startPose)
                        .forward(27,
                                SampleMecanumDrive.getVelocityConstraint(15,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(15))
                        .strafeLeft(24,
                                SampleMecanumDrive.getVelocityConstraint(15,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(15))
                        .build();
                drive.followTrajectorySequence(Park1);
            }
            else if (tagOfInterest.id==14){
                //to second spot
                TrajectorySequence Park2 = drive.trajectorySequenceBuilder(startPose)
                        .forward(27,
                                SampleMecanumDrive.getVelocityConstraint(15,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(15))
                        .build();
                drive.followTrajectorySequence(Park2);
            }
            else if(tagOfInterest.id==19) {
                //to third spot
                TrajectorySequence Park3 = drive.trajectorySequenceBuilder(startPose)
                        .forward(27,
                                SampleMecanumDrive.getVelocityConstraint(15,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(15))
                        .strafeRight(24,
                                SampleMecanumDrive.getVelocityConstraint(15,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(15))
                        .build();
                drive.followTrajectorySequence(Park3);
            }
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