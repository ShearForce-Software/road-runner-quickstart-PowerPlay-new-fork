package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.ArmControlRR;
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
@Autonomous(name = "RR Left Auto Edit")
public class RR_Left_Auto_Edit extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166;
    AprilTagDetection tagOfInterest = null;
    public static double startX = -36;            // added start variable
    public static double startY = -64.5;          // added start variable
    public static double startHeading = -90;      // added start variable
    public static double stackY = -12.5;
    public static double stackX = -58.5;
    //public static double junctionX = -26;
    //public static double junctionY = -6;
    public static double junction1X = -26.5;          // added for to ID specific junction X
    public static double junction1Y = -7.5;           // added for to ID specific junction Y
    public static double junction1Heading = -135;     // added for to ID specific junction Heading
    //public static double junction2X = -26;            // added for to ID specific junction X
    //public static double junction2Y = -6;             // added for to ID specific junction Y
    //public static double junction2Heading = -135;     // added for to ID specific junction Heading
    public static double numConesStack = 3;
    public static double toFirstConeVel = 50;
    public static double toStackVel = 25;
    public static double toHighVel = 25;

    @Override
    public void runOpMode() throws InterruptedException {
        ArmControlRR armControl = new ArmControlRR(false, false, false, this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(startX, startY, Math.toRadians(startHeading));  // added to replace line above
        Vector2d junction1Vec = new Vector2d(junction1X, junction1Y);
        Pose2d junction1Pos = new Pose2d(junction1X,junction1Y, Math.toRadians(junction1Heading));
        Pose2d stackPos = new Pose2d(stackX, stackY, Math.toRadians(-180));

        String step;

        drive.setPoseEstimate(startPose);
        armControl.Init(hardwareMap);
        armControl.StartPosition(null, false);

        TrajectorySequence FirstCone = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-36, -20), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(toFirstConeVel,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .splineToSplineHeading(new Pose2d(junction1Vec.getX()-(2*(1/Math.sqrt(2))), junction1Vec.getY()-(2*(1/Math.sqrt(2))), Math.toRadians(-135)), Math.toRadians(45), //TODO: check signal cone deflection with and without sqrt code
                        SampleMecanumDrive.getVelocityConstraint(toFirstConeVel,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35))
                .splineToConstantHeading(junction1Vec, Math.toRadians(45),
                        SampleMecanumDrive.getVelocityConstraint(toFirstConeVel -20,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH), // TODO: check velocity
                        SampleMecanumDrive.getAccelerationConstraint(35))
                .build();

       // TrajectorySequence ToStack = drive.trajectorySequenceBuilder(drive.ce())  // use getPoseEstimate vs. junction1Pos to minimize error


        AprilTags();
        waitForStart();
        if (isStopRequested()) return;
        if (opModeIsActive()) {
            
            drive.followTrajectorySequenceAsync(FirstCone);
            //SlidesToHighHardCode(armControl, drive);        //is this needed since GoTohigh180 has same code?
            armControl.GoToHigh180(drive);
            armControl.WaitForTrajectoryToFinish(drive);
            //armControl.SpecialSleep(drive, 1350);
            armControl.openClaw();

            armControl.STACK_POS = 350;
            for (int i = 0; i < numConesStack; i++){
                //armControl.openClaw();                                // open claw
                step = "one - command open claw";
                telemetry.addData("step: ", step);                 // add telemetry status
                telemetry.update();                                   // display on screen

                TrajectorySequence ToStack = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(-38, stackY, Math.toRadians(180)), Math.toRadians(180),
                                SampleMecanumDrive.getVelocityConstraint(toStackVel,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(toStackVel))
                        .splineToLinearHeading(stackPos, Math.toRadians(180),
                                SampleMecanumDrive.getVelocityConstraint(toStackVel,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(toStackVel))
                        .build();

                drive.followTrajectorySequenceAsync(ToStack);         // drive toward stack without using distance sensor correction

                step = "two - command movement to stack";
                telemetry.addData("step: ", step);
                telemetry.update();
                armControl.SpecialSleep(drive, 450);       // wait 450 msec to start claw movement when moving away from junction

                armControl.closeClaw(); //god only knows why we need this here but it doesn't like to close the claw so TODO:re-check if needed

                armControl.ReadyToGrabFromStack(drive);              // complete arm movement to be ready to pick up cone

                step = "two a - position arm to pickup cone from stack";
                telemetry.addData("step: ", step);
                telemetry.update();
                armControl.WaitForTrajectoryToFinish(drive);         // complete drive to stack
                step = "two b - finished drive to stack - ready to pickup";
                telemetry.addData("step: ", step);
                telemetry.update();

                armControl.GrabFromStack(drive);                    // grip and lift cone from stack

                step = "five - command cone pickup from top of stack";
                telemetry.addData("step: ", step);
                telemetry.update();
                armControl.SpecialSleep(drive, 200);      // wait 200 to allow slides to raise

                TrajectorySequence ToHighJunction = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .setReversed(true)
                        .strafeTo(new Vector2d(-44,-12),
                                SampleMecanumDrive.getVelocityConstraint(toHighVel,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(toHighVel))
                        .splineToSplineHeading(new Pose2d(-36, -12, Math.toRadians(-135)), Math.toRadians(0),
                                SampleMecanumDrive.getVelocityConstraint(toHighVel,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(toHighVel))
                        .splineToConstantHeading(junction1Vec, Math.toRadians(45),  // changed to junction1Vec
                                SampleMecanumDrive.getVelocityConstraint(toHighVel,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(toHighVel))
                        .build();
                drive.followTrajectorySequenceAsync(ToHighJunction);        // drive to high junction
                //SlidesToHighHardCode(armControl, drive);                 // sets conditions to raise the slides to high position but does not command it
                armControl.autoArmToHigh(drive);                            // actually commands and executes arm movement for delivery
                armControl.SpecialSleepTraj(drive, 1850);             // wait 1.85 sec then open claw - there is no wait for trajectory to complete
                if (i == 1 || i == 2) armControl.STACK_POS -= 175;
                if (i > 2) armControl.STACK_POS = armControl.START_POS;     //start pos = 10
                armControl.openClaw();
                // set position for next cone pickup
            }
            //~~~~~~~~~~parking testing note: test between pose estimate and actual pose

            if (tagOfInterest.id==11){
                //to first spot
                TrajectorySequence Park1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(-48, stackPos.getY(), Math.toRadians(180)), Math.toRadians(180),
                                SampleMecanumDrive.getVelocityConstraint(50,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(30))
                        .forward(10,
                                SampleMecanumDrive.getVelocityConstraint(50,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(30))
                        .build();
                drive.followTrajectorySequenceAsync(Park1);
            }
            else if (tagOfInterest.id==14){
                //to second spot
                TrajectorySequence Park2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .setReversed(false)
                        .splineToLinearHeading(new Pose2d(-36, stackPos.getY(), Math.toRadians(180)), Math.toRadians(-135),
                                SampleMecanumDrive.getVelocityConstraint(40,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(30))
                        .build();
                drive.followTrajectorySequenceAsync(Park2);
            }
            else if(tagOfInterest.id==19) {
                //to third spot
                Pose2d temp = drive.getPoseEstimate();
                TrajectorySequence Park3 = drive.trajectorySequenceBuilder(temp)
                        .setReversed(false)
                        .forward(2.5,
                                SampleMecanumDrive.getVelocityConstraint(40,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(30))
                        .splineToSplineHeading(new Pose2d(temp.getX()-(6*(1/Math.sqrt(2))), temp.getY()-(6*(1/Math.sqrt(2))), Math.toRadians(180)), Math.toRadians(-135),
                                SampleMecanumDrive.getVelocityConstraint(40,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(30))
                        .splineToConstantHeading(new Vector2d(-24, stackPos.getY()), Math.toRadians(0),
                                SampleMecanumDrive.getVelocityConstraint(40,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(30))
                        .splineToConstantHeading(new Vector2d(-12, stackPos.getY()), Math.toRadians(0),
                                SampleMecanumDrive.getVelocityConstraint(55,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(45))
                        .build();
                drive.followTrajectorySequenceAsync(Park3);
            }
            armControl.SpecialSleep(drive, 1000); //~~~~~EXPERIMENT with this time
            armControl.ReturnFromHigh(drive);
            //armControl.closeClaw();
            armControl.liftWrist.setPosition(.6);
            armControl.WaitForTrajectoryToFinish(drive);
            step = "six - finished parking";
            telemetry.addData("step: ", step);
            telemetry.update();


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