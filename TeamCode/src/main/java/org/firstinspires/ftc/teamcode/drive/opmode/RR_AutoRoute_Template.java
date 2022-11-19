package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.CustomModelWebcam;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;

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
@Autonomous(name = "AutoRoute")
public class RR_AutoRoute_Template extends LinearOpMode {
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
    private CustomModelWebcam customModelWebcam = new CustomModelWebcam();
    private static final String TFOD_MODEL_ASSET = "model_20221103_190954.tflite";
    private static final String[] LABELS = {
            "Checkered1",
            "Logo3",
            "Squig2"
    };
    private static final String VUFORIA_KEY =
            "AYjyN7v/////AAABmZkzDnVgYED5uG0oVjDFNPU/IVXNIkVpEj5VY0d385xq3MN8tk1R/zBRduVWZPRSZBzSAyuJoJpgI79HeoJoMFQd/p0ZcytKFDckit+NkdDBJaBa1RXvpH8JufADNrmBkF8WhyUkFrROxOoCRsq1/TFrGaxicoJahSo6XUIk0YTfvIp5vJjzFWruq+IiAoWzChKdEA3GIEZE9Fufr2omudFjgF/k5JkIzQU01ou6Nrj59p0sndgCl+tIFKsDY/+WW28UpdFRz4lyR3apWeS+rtflqR52ofXjCF7sh08J7ZQ4Cqblwq2dOb0r/MoabLLxdSJdW15MH12ZDNNTxWttAkwgRxLdiiK44ogzcDByFAtb";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    public String label;

    public void initWebcam() {
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0, 16.0 / 9.0);
        }
        /* Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
    }

    public void tfodDetection() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Objects Detected", updatedRecognitions.size());

                // step through the list of recognitions and display image position/size information for each one
                // Note: "Image number" refers to the randomized image orientation/number
                for (Recognition recognition : updatedRecognitions) {
                    double col = (recognition.getLeft() + recognition.getRight()) / 2;
                    double row = (recognition.getTop() + recognition.getBottom()) / 2;
                    double width = Math.abs(recognition.getRight() - recognition.getLeft());
                    double height = Math.abs(recognition.getTop() - recognition.getBottom());

                    telemetry.addData("", " ");
                    label = recognition.getLabel();
                    telemetry.addData("Image", "%s (%.0f %% Conf.)", label, recognition.getConfidence() * 100);
                    telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                    telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);
                }
                telemetry.update();
            }
        }
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.65f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

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

        initWebcam();
        while (!isStarted()) {
            tfodDetection();
        }
        // Starting position of robot on field
        Pose2d startPose = new Pose2d(-36, 60, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);
        waitForStart();
        if (isStopRequested()) return;
        if (opModeIsActive()) {
            if (label == "Checkered1"){
                //to first spot
                TrajectorySequence Park1 = drive.trajectorySequenceBuilder(startPose)
                        .forward(26)
                        .strafeLeft(24)
                        .build();
                drive.followTrajectorySequence(Park1);
            }
            else if(label == "Logo3") {
                //to third spot
                TrajectorySequence Park3 = drive.trajectorySequenceBuilder(startPose)
                        .forward(26)
                        .strafeRight(24)
                        .build();
                drive.followTrajectorySequence(Park3);
            }
            else{
                //to second spot
                TrajectorySequence Park2 = drive.trajectorySequenceBuilder(startPose)
                        .forward(36)
                        .build();
                drive.followTrajectorySequence(Park2);
            }
        }
    }
}