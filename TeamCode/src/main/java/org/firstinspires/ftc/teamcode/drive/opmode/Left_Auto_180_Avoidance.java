package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
//@Disabled
@Config
@Autonomous(name = "EXPERIMENT Autonomous Left")
public class Left_Auto_180_Avoidance extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166;
    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {
        ArmControl armControl = new ArmControl(false, false, this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        double stackY = -14.26;
        armControl.STACK_POS = 550;
        Pose2d startPose = new Pose2d(-36, -64.5, Math.toRadians(-90));

        Vector2d junctionVec = new Vector2d(-26, -6);
        Pose2d junctionPos = new Pose2d(-26,-6, Math.toRadians(-135));
        Vector2d junctionTwoVec = new Vector2d(-5,-22);
        Pose2d junctionTwoPos = new Pose2d(-5,-22, Math.toRadians(135));

        Pose2d almostStackPos = new Pose2d(-58, stackY, Math.toRadians(-180));
        Vector2d realStackVec;
        TrajectorySequence ToRealStack;
        TrajectorySequence ToHighJunction;
        drive.setPoseEstimate(startPose);
        armControl.Init(hardwareMap);
        armControl.StartPosition(null, false);

        TrajectorySequence FirstCone = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-36, -20), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(55,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .splineToSplineHeading(new Pose2d(junctionVec.getX()-(2*(1/Math.sqrt(2))), junctionVec.getY()-(2*(1/Math.sqrt(2))), Math.toRadians(-135)), Math.toRadians(45),
                        SampleMecanumDrive.getVelocityConstraint(55,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35))
                .splineToConstantHeading(junctionVec, Math.toRadians(45),
                        SampleMecanumDrive.getVelocityConstraint(35,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35))

//                .splineToConstantHeading(new Vector2d(-36, -22), Math.toRadians(90),
//                        SampleMecanumDrive.getVelocityConstraint(55,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(55))
//                .splineToSplineHeading(new Pose2d(-36, -16.5, Math.toRadians(-135)), Math.toRadians(90),
//                        SampleMecanumDrive.getVelocityConstraint(35,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(10))
//                .splineToConstantHeading(junctionVec, Math.toRadians(45),
//                        SampleMecanumDrive.getVelocityConstraint(45,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(10))
                .build();

        TrajectorySequence ToAlmostStackOne = drive.trajectorySequenceBuilder(junctionPos)
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-38, -14.26, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(25,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .splineToLinearHeading(almostStackPos, Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(25,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .build();

        TrajectorySequence ToAlmostStackTwo = drive.trajectorySequenceBuilder(junctionTwoPos)
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-16,-14.26,Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(25,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .splineToSplineHeading(new Pose2d(-50,-14.26,Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(25,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .splineToLinearHeading(almostStackPos, Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(25,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .build();

        AprilTags();
        waitForStart();
        if (isStopRequested()) return;
        if (opModeIsActive()) {
            drive.followTrajectorySequenceAsync(FirstCone);
            SlidesToHighHardCode(armControl, drive);
            armControl.GoToHigh180(drive);
            armControl.WaitForTrajectoryToFinish(drive);
            armControl.openClaw();

            //UNCOMMENT ONLY after test to make sure the new method works


            for (int i = 0; i < 1; i++){
                armControl.openClaw();
                if(i==0){
                    drive.followTrajectorySequenceAsync(ToAlmostStackOne);
                }
                else{
                    drive.followTrajectorySequenceAsync(ToAlmostStackTwo);
                }
                armControl.SpecialSleep(drive, 450);
                armControl.closeClaw(); //god only knows why we need this here but it doesn't like to close the claw so
                armControl.ReadyToGrabFromStack(drive);
                armControl.WaitForTrajectoryToFinish(drive);

                armControl.FindConeCenter();

                realStackVec = new Vector2d(drive.getPoseEstimate().getX() - armControl.forwardLG, drive.getPoseEstimate().getY() - armControl.shiftLG);
                ToRealStack = drive.trajectorySequenceBuilder(almostStackPos)
                        .splineToConstantHeading(realStackVec, Math.toRadians(180),
                                SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(20))
                        .build();
                drive.followTrajectorySequenceAsync(ToRealStack);
                armControl.WaitForTrajectoryToFinish(drive);
                armControl.GrabFromStack(drive);
                armControl.SpecialSleep(drive, 200);
                ToHighJunction = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(-20, -13), Math.toRadians(0),
                                SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(50))
                        .splineToSplineHeading(new Pose2d(junctionTwoVec.getX()-(4*(1/Math.sqrt(2))), junctionTwoVec.getY()+(4*(1/Math.sqrt(2))), Math.toRadians(135)), Math.toRadians(-45),
                                SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(50))
                        .splineToConstantHeading(junctionTwoVec, Math.toRadians(-45),
                                SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(50))
                        .build();
                drive.followTrajectorySequenceAsync(ToHighJunction);
                SlidesToHighHardCode(armControl, drive); //love it sm
                armControl.autoArmToHigh(drive);
                armControl.SpecialSleep(drive, 2850); //~~~~~~~~~~~EDIT THIS TIME FOR DROPPING THE CONE AND RESETTING ETC(it will need to change)
                armControl.STACK_POS -= 125;
            }
            armControl.openClaw();
            /*

            if (tagOfInterest.id==11){
                //to first spot
                TrajectorySequence Park1 = drive.trajectorySequenceBuilder(junctionTwoPos)
                        .setReversed(false)
                        .forward(1.5,
                                SampleMecanumDrive.getVelocityConstraint(50,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(50))
                        .splineToSplineHeading(new Pose2d(-24, -12, Math.toRadians(180)), Math.toRadians(180),
                                SampleMecanumDrive.getVelocityConstraint(50,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(50))
                        .splineToConstantHeading(new Vector2d(-58, -12), Math.toRadians(180),
                                SampleMecanumDrive.getVelocityConstraint(50,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(50))
                        .build();
                drive.followTrajectorySequenceAsync(Park1);
            }
            else if (tagOfInterest.id==14){
                //to second spot
                TrajectorySequence Park2 = drive.trajectorySequenceBuilder(junctionTwoPos)
                        .setReversed(false)
                        .forward(1.5,
                                SampleMecanumDrive.getVelocityConstraint(50,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(50))
                        .splineToSplineHeading(new Pose2d(-24, -12, Math.toRadians(180)), Math.toRadians(180),
                                SampleMecanumDrive.getVelocityConstraint(50,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(50))
                        .splineToConstantHeading(new Vector2d(-36, -12), Math.toRadians(180),
                                SampleMecanumDrive.getVelocityConstraint(50,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(50))
                        .build();
                drive.followTrajectorySequenceAsync(Park2);
            }
            else if(tagOfInterest.id==19) {
                //to third spot
                TrajectorySequence Park3 = drive.trajectorySequenceBuilder(junctionTwoPos)
                        .setReversed(false)
                        .forward(1.5,
                                SampleMecanumDrive.getVelocityConstraint(50,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(50))
                        .splineToSplineHeading(new Pose2d(-12, -12, Math.toRadians(180)), Math.toRadians(135),
                                SampleMecanumDrive.getVelocityConstraint(60,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(60))
                        .build();
                drive.followTrajectorySequenceAsync(Park3);
            }
            armControl.SpecialSleep(drive, 1000);
            armControl.ReturnFromHigh(drive);
            armControl.closeClaw();
            armControl.liftWrist.setPosition(1);
            armControl.WaitForTrajectoryToFinish(drive);

             */

        }
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

    private void SlidesToHighHardCode(ArmControl armControl, SampleMecanumDrive drive) {
        armControl.slideOne.setTargetPosition(armControl.HIGH_POS);
        armControl.slideTwo.setTargetPosition(armControl.HIGH_POS);
        armControl.slideOne.setPower(armControl.ARM_POWER);
        armControl.slideTwo.setPower(armControl.ARM_POWER);
//        armControl.WaitForSlides(drive);
//        armControl.slideOne.setPower(0);
//        armControl.slideTwo.setPower(0);
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