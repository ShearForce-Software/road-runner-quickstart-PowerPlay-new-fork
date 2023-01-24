package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
@Autonomous(name = "COOl PRETURN Cone Stack Auto Right")
public class Cone_Stack_Right_Auto_PreTurn extends LinearOpMode {
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
        double stackY = -12;
        armControl.STACK_POS = 550;
        Pose2d startPose = new Pose2d(36, -64.5, Math.toRadians(90));
        Vector2d junctionVec = new Vector2d(26.3,-7.3);
        Pose2d junctionPos = new Pose2d(26.3,-7.3, Math.toRadians(-45));
        Pose2d NEWjunctionPos = new Pose2d(29.8, -5,  Math.toRadians(-25));
        Pose2d stackPos = new Pose2d(63.5, stackY, Math.toRadians(0));
        Pose2d linePos = new Pose2d(40, stackY, Math.toRadians(0));
        drive.setPoseEstimate(startPose);
        armControl.Init(hardwareMap);
        armControl.StartPosition(null);

        TrajectorySequence FirstCone = drive.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(36,-56),
                        SampleMecanumDrive.getVelocityConstraint(60,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(60))
                .splineToSplineHeading(new Pose2d(36, -28, 0), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(60,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(60))
                .splineToConstantHeading(new Vector2d(36, -24), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(60,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(60))
                .splineToSplineHeading(new Pose2d(36, -16.5, Math.toRadians(-45)), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(45,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(45))
                .splineToConstantHeading(junctionVec, Math.toRadians(135),
                        SampleMecanumDrive.getVelocityConstraint(45,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(45))
                .build();

        TrajectorySequence OGToStack = drive.trajectorySequenceBuilder(junctionPos)
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(38, -12, Math.toRadians(0)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(25,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .splineToLinearHeading(stackPos, Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(25,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .build();


//        TrajectorySequence NEWToStack = drive.trajectorySequenceBuilder(NEWjunctionPos)
//                .setReversed(false)
//                .splineToSplineHeading(stackPos , Math.toRadians(0),
//                        SampleMecanumDrive.getVelocityConstraint(25,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(25))
//                .build();

        TrajectorySequence ToHighJunction = drive.trajectorySequenceBuilder(stackPos)
                .setReversed(true)
                .strafeTo(new Vector2d(44,-12),
                        SampleMecanumDrive.getVelocityConstraint(20,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(20))
                .splineToSplineHeading(new Pose2d(36, -12, Math.toRadians(-45)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(20,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(20))
                .splineToConstantHeading(junctionVec, Math.toRadians(135),
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

            //**************ADDED HARD CODE FOR SLIDE MOVEMENTS**************//
            SlidesToStowHardCode(armControl, drive);
            armControl.StowCone(drive);
            armControl.GoToHigh(drive);
            SlidesToHighHardCode(armControl, drive);
            armControl.WaitForTrajectoryToFinish(drive);

            for (int i = 0; i < 1; i++){
                armControl.openClaw();
                drive.followTrajectorySequenceAsync(OGToStack);
                armControl.SpecialSleep(drive, 450);
                armControl.closeClaw(); //who knows why we need this here but it doesn't like to close the claw so
                armControl.ReadyToGrabFromStack(drive);
                armControl.SpecialSleep(drive, 800);
                armControl.GrabFromStack(drive);
                drive.followTrajectorySequenceAsync(ToHighJunction);
                //armControl.SpecialSleep(drive, 250);
                SlidesToStowHardCode(armControl, drive); //we love having trust issues with the slides
                armControl.StowCone(drive);
                SlidesToHighHardCode(armControl, drive); //love it sm
                armControl.GoToHigh(drive);
                armControl.SpecialSleep(drive, 1700);
//                armControl.WaitForTrajectoryToFinish(drive);
                //stackY += 1;
                //stackPos = new Pose2d(63.5, stackY, Math.toRadians(0));
                armControl.STACK_POS -= 125;
            }


            armControl.openClaw();
            if (tagOfInterest.id==11){
                //to first spot
                TrajectorySequence Park1 = drive.trajectorySequenceBuilder(FirstCone.end())
                        .splineToSplineHeading(new Pose2d(36, -24, Math.toRadians(-90)), Math.toRadians(-90),
                                SampleMecanumDrive.getVelocityConstraint(40,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(40))
                        .forward(11,
                                SampleMecanumDrive.getVelocityConstraint(40,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(40))
                        .strafeRight(22,
                                SampleMecanumDrive.getVelocityConstraint(50,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(50))
                        .build();
                drive.followTrajectorySequenceAsync(Park1);
            }
            else if (tagOfInterest.id==14){
                //to second spot
                TrajectorySequence Park2 = drive.trajectorySequenceBuilder(FirstCone.end())
                        .splineToSplineHeading(new Pose2d(34.5, -14, Math.toRadians(-90)), Math.toRadians(-90),
                                SampleMecanumDrive.getVelocityConstraint(40,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(40))
//                        .forward(11,
//                                SampleMecanumDrive.getVelocityConstraint(30,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
//                                SampleMecanumDrive.getAccelerationConstraint(30))
                        .build();
                drive.followTrajectorySequenceAsync(Park2);
            }
            else if(tagOfInterest.id==19) {
                //to third spot
                TrajectorySequence Park3 = drive.trajectorySequenceBuilder(FirstCone.end())
                        .splineToSplineHeading(new Pose2d(36, -24, Math.toRadians(-90)), Math.toRadians(-90),
                                SampleMecanumDrive.getVelocityConstraint(40,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(40))
                        .forward(11,
                                SampleMecanumDrive.getVelocityConstraint(40,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(40))
                        .strafeLeft(24,
                                SampleMecanumDrive.getVelocityConstraint(50,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(50))
                        .build();
                drive.followTrajectorySequenceAsync(Park3);
            }
            armControl.SpecialSleep(drive, 1000);
            armControl.ReturnFromHigh(drive);
            armControl.closeClaw();
            armControl.WaitForTrajectoryToFinish(drive);
        }
    }

    private void SlidesToHighHardCode(ArmControl armControl, SampleMecanumDrive drive) {
        armControl.slideOne.setTargetPosition(armControl.HIGH_POS);
        armControl.slideTwo.setTargetPosition(armControl.HIGH_POS);
        armControl.slideOne.setPower(armControl.ARM_POWER);
        armControl.slideTwo.setPower(armControl.ARM_POWER);
        armControl.WaitForSlides(drive);
        armControl.slideOne.setPower(0);
        armControl.slideTwo.setPower(0);
    }

    private void SlidesToStowHardCode(ArmControl armControl, SampleMecanumDrive drive) {
        armControl.slideOne.setTargetPosition(armControl.STOW_POS);
        armControl.slideTwo.setTargetPosition(armControl.STOW_POS);
        armControl.slideOne.setPower(armControl.ARM_POWER);
        armControl.slideTwo.setPower(armControl.ARM_POWER);
        armControl.WaitForSlides(drive);
        armControl.slideOne.setPower(0);
        armControl.slideTwo.setPower(0);
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