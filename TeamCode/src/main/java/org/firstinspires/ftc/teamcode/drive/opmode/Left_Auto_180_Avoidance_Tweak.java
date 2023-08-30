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
@Disabled
@Config
@Autonomous(name = "AVOIDANCE Left Autonomous")
public class Left_Auto_180_Avoidance_Tweak extends LinearOpMode {
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

        Vector2d junctionVec = new Vector2d(-28.5, -7);
        Pose2d junctionPos = new Pose2d(-28.5,-7, Math.toRadians(-135));
        Vector2d junctionTwoVec = new Vector2d(-4,-21.5);
        Pose2d junctionTwoPos = new Pose2d(-5,-23, Math.toRadians(135));
        Pose2d stackEstimate = new Pose2d(0,0,0);

        //telemetry positions only
        Pose2d firstJuncEstimate = new Pose2d(0,0,0);
        Pose2d secondJuncEstimate = new Pose2d(0,0,0);


        Pose2d almostStackPos = new Pose2d(-57, stackY, Math.toRadians(-180));
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
                .build();

        TrajectorySequence ToAlmostStackOne = drive.trajectorySequenceBuilder(junctionPos)
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-38, stackY, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(25,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .splineToLinearHeading(almostStackPos, Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(25,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .build();

        TrajectorySequence ToAlmostStackTwo;


        AprilTags();
        waitForStart();
        if (isStopRequested()) return;
        if (opModeIsActive()) {
            drive.followTrajectorySequenceAsync(FirstCone);
            SlidesToHighHardCode(armControl, drive);
            armControl.GoToHigh180(drive);
            armControl.WaitForTrajectoryToFinish(drive);
            armControl.SpecialSleep(drive, 100);
            armControl.openClaw();

            firstJuncEstimate = drive.getPoseEstimate();

            telemetry.addData("First Junction Pose ~ (X: ","%.2f  %.2f",firstJuncEstimate.getX(), firstJuncEstimate.getY());
            telemetry.update();

            for (int i = 0; i < 3; i++){
                armControl.openClaw();
                if(i==0){
                    //RUNS THIS AFTER THE PRELOADED CONE HAS BEEN DROPPED ONLY ONCE
                    drive.followTrajectorySequenceAsync(ToAlmostStackOne);
                }
                else{
                    //RUNS THIS AFTER THE FIRST CONE FROM THE STACK HAS BEEN RELEASED
                    almostStackPos = new Pose2d(-58, stackEstimate.getY(), Math.toRadians(-180));
                    ToAlmostStackTwo = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .setReversed(false)
                            .splineToSplineHeading(new Pose2d(-16,stackEstimate.getY(),Math.toRadians(180)), Math.toRadians(180),
                                    SampleMecanumDrive.getVelocityConstraint(25,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(35))
                            .splineToSplineHeading(new Pose2d(-50,stackEstimate.getY(),Math.toRadians(180)), Math.toRadians(180),
                                    SampleMecanumDrive.getVelocityConstraint(55,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(35))
                            .splineToLinearHeading(almostStackPos, Math.toRadians(180),
                                    SampleMecanumDrive.getVelocityConstraint(25,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(25))
                            .build();
                    drive.followTrajectorySequenceAsync(ToAlmostStackTwo);
                }
                armControl.SpecialSleep(drive, 450);
                armControl.closeClaw(); //god only knows why we need this here but it doesn't like to close the claw so
                armControl.ReadyToGrabFromStack(drive);
                armControl.WaitForTrajectoryToFinish(drive);

                armControl.FindConeCenter();

                realStackVec = new Vector2d(drive.getPoseEstimate().getX() - armControl.forwardLG, drive.getPoseEstimate().getY() - armControl.shiftLG);
                ToRealStack = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .splineToConstantHeading(realStackVec, Math.toRadians(180),
                                SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(20))
                        .build();
                drive.followTrajectorySequenceAsync(ToRealStack);
                armControl.WaitForTrajectoryToFinish(drive);

                stackEstimate = drive.getPoseEstimate();
//                if(i==0){
//                    temp = drive.getPoseEstimate();
//                }
                telemetry.clearAll();
                telemetry.addData("1st Junc Estimate ~ (X: ","%.2f  %.2f",firstJuncEstimate.getX(), firstJuncEstimate.getY());
                telemetry.addData("Stack Estimate Pose ~ (X: ","%.2f  %.2f",stackEstimate.getX(), stackEstimate.getY());
                telemetry.addData("2nd Junc Estimate ~ (X: ","%.2f  %.2f", secondJuncEstimate.getX(), secondJuncEstimate.getY());
                telemetry.update();

                armControl.GrabFromStack(drive);
                armControl.SpecialSleep(drive, 200);
                ToHighJunction = drive.trajectorySequenceBuilder(stackEstimate)
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(-24, stackEstimate.getY()), Math.toRadians(0),
                                SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(35))
//                        .forward(-38,
//                                SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                                SampleMecanumDrive.getAccelerationConstraint(35))
                        //.splineToSplineHeading(new Pose2d(-14, stackEstimate.getY(), Math.toRadians(-45)), Math.toRadians(0),
                        //        SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        //        SampleMecanumDrive.getAccelerationConstraint(25))
                        .splineToSplineHeading(junctionTwoPos, Math.toRadians(-45),
                                SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(25))
                        .build();
                drive.followTrajectorySequenceAsync(ToHighJunction);
                 SlidesToHighHardCode(armControl, drive); //love it sm
                armControl.autoArmToHigh(drive);
                armControl.SpecialSleep(drive, 2100); //~~~~~~~~~~~EDIT THIS TIME FOR DROPPING THE CONE AND RESETTING ETC
                armControl.STACK_POS -= 125;

                secondJuncEstimate = drive.getPoseEstimate();
                telemetry.clearAll();
                telemetry.addData("1st Junc Estimate ~ (X: ","%.2f  %.2f",firstJuncEstimate.getX(), firstJuncEstimate.getY());
                telemetry.addData("Stack Estimate Pose ~ (X: ","%.2f  %.2f",stackEstimate.getX(), stackEstimate.getY());
                telemetry.addData("2nd Junc Estimate ~ (X: ","%.2f  %.2f", secondJuncEstimate.getX(), secondJuncEstimate.getY());
                telemetry.update();
            }
            armControl.openClaw();

            //~~~~~~~~~~parking testing note: test between pose estimate and actual pose

            if (tagOfInterest.id==11){
                //to first spot
                TrajectorySequence Park1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .setReversed(false)
                        .forward(1.5,
                                SampleMecanumDrive.getVelocityConstraint(55,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(40))
                        .splineToSplineHeading(new Pose2d(-24, stackEstimate.getY(), Math.toRadians(180)), Math.toRadians(180),
                                SampleMecanumDrive.getVelocityConstraint(55,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(50))
                        .splineToLinearHeading(new Pose2d(-56.75, stackEstimate.getY(), Math.toRadians(180)), Math.toRadians(180),
                                SampleMecanumDrive.getVelocityConstraint(55,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(40))
                        .build();
                drive.followTrajectorySequenceAsync(Park1);
            }

            else if (tagOfInterest.id==14){
                //to second spot
                TrajectorySequence Park2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .setReversed(false)
                        .forward(1.5,
                                SampleMecanumDrive.getVelocityConstraint(55,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(30))
                        .splineToSplineHeading(new Pose2d(-24, stackEstimate.getY(), Math.toRadians(180)), Math.toRadians(180),
                                SampleMecanumDrive.getVelocityConstraint(55,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(50))
                        .splineToConstantHeading(new Vector2d(-30, stackEstimate.getY()), Math.toRadians(180),
                                SampleMecanumDrive.getVelocityConstraint(55,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(50))
                        .splineToConstantHeading(new Vector2d(-36, stackEstimate.getY()), Math.toRadians(180),
                                SampleMecanumDrive.getVelocityConstraint(55,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(20))
                        .build();
                drive.followTrajectorySequenceAsync(Park2);
            }
            else if(tagOfInterest.id==19) {
                //to third spot
                TrajectorySequence Park3 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .setReversed(false)
                        .forward(1.5,
                                SampleMecanumDrive.getVelocityConstraint(55,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(30))
                        .splineToSplineHeading(new Pose2d(-12, stackEstimate.getY(), Math.toRadians(180)), Math.toRadians(135),
                                SampleMecanumDrive.getVelocityConstraint(55,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(30))
                        .build();
                drive.followTrajectorySequenceAsync(Park3);
            }
            armControl.SpecialSleep(drive, 1000); //~~~~~EXPERIMENT with this time
            armControl.ReturnFromHigh(drive);
            armControl.closeClaw();
            armControl.SpecialSleep(null, 180);
            armControl.liftWrist.setPosition(1);
            armControl.WaitForTrajectoryToFinish(drive);

             //*/

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