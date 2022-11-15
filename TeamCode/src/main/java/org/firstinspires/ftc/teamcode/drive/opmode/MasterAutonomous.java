/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drive.CustomModelWebcam;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;


@TeleOp(name="Master Autonomous")
//@Disabled
public class MasterAutonomous extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private CustomModelWebcam customModelWebcam = new CustomModelWebcam();
    private static final String TFOD_MODEL_ASSET = "model_20221103_190954.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";


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

    @Override
    public void runOpMode() {
        initWebcam();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        while (!isStarted()) {
            tfodDetection();
        }
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                //INITIAL AUTO ROUTES
                //single preloaded Right
                drive.followTrajectorySequence(
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 36, Math.toRadians(-90)))
                                .splineToSplineHeading(new Pose2d(-22, 35, Math.toRadians(-45)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(-8, 32, Math.toRadians(-45)), Math.toRadians(-45))
                                //jacob servo and motor code added in using markers
                                .build()
                );
                //single preloaded Left
                /*
                drive.followTrajectorySequence(
                        drive.trajectorySequenceBuilder(new Pose2d(36, 36, Math.toRadians(-90)))
                                .splineToSplineHeading(new Pose2d(22, 35, Math.toRadians(-135)), Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(8, 32, Math.toRadians(-135)), Math.toRadians(-145))
                                //jacob servo and motor code added in using markers
                                .build()
                );
                */
                //PARKING BASED OFF OF SIGNAL - Right Side
                if (label.equals("Checkered1")){
                    //to first spot
                    drive.followTrajectorySequence(
                            drive.trajectorySequenceBuilder(new Pose2d(36, 36, Math.toRadians(-45)))
                                    .lineToLinearHeading(new Pose2d(-12, 36, Math.toRadians(90)))
                                    .build()
                    );
                }
                else if(label.equals("Logo3")) {
                    //to third spot
                    drive.followTrajectorySequence(
                            drive.trajectorySequenceBuilder(new Pose2d(36, 36, Math.toRadians(-45)))
                                    .splineToSplineHeading(new Pose2d(-24, 36, Math.toRadians(0)), Math.toRadians(180))
                                    .lineToLinearHeading(new Pose2d(-60, 36, Math.toRadians(0)))
                                    .build()
                    );
                }
                else{
                    //to second spot
                    drive.followTrajectorySequence(
                            drive.trajectorySequenceBuilder(new Pose2d(36, 36, Math.toRadians(-45)))
                                    .setReversed(true)
                                    .lineToLinearHeading(new Pose2d(-36, 36, Math.toRadians(0)))
                                    .build()
                    );
                }
            }
        }
    }
    public void tfodDetection () {
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
    public void initWebcam () {
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0, 16.0 / 9.0);
        }
        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
    }
    private void initVuforia () {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    private void initTfod () {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}
