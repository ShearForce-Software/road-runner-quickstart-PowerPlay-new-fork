package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.drive.CustomModelWebcam;

import java.util.concurrent.atomic.AtomicInteger;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@TeleOp(name = "Auto Route Left Far")
public class Auto_Route_Left_Far extends LinearOpMode {
    private CustomModelWebcam customModelWebcam = new CustomModelWebcam();
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //Ben testing stuff
        /*
        customModelWebcam.initWebcam();
        customModelWebcam.runOpModeActive();
        String sleeveLabel = customModelWebcam.label;
        AtomicInteger parkingSpot = new AtomicInteger();
        */
        waitForStart();

        if (isStopRequested()) return;

        //sleep(2000);

        drive.followTrajectorySequence(
                drive.trajectorySequenceBuilder(new Pose2d(36, 36, Math.toRadians(-90)))
                        //More ben test
                        /*
                        .addDisplacementMarker(() -> {
                            if (sleeveLabel.equals("Checkered1")) {
                                parkingSpot.set(1);
                            }
                            else if (sleeveLabel.equals("Logo3")){
                                parkingSpot.set(3);
                            }else{
                                parkingSpot.set(2);
                            }
        })
                         */
                        .splineToLinearHeading(new Pose2d(50, 12, Math.toRadians(-180)), Math.toRadians(0))
                        //1st cone
                        .splineToLinearHeading(new Pose2d(31, 7, Math.toRadians(135)), Math.toRadians(135))
                        .waitSeconds(.75)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(58, 12, Math.toRadians(-180)), Math.toRadians(0))
                        .waitSeconds(.75)
                        .setReversed(false)
                        //2nd cone
                        .splineToLinearHeading(new Pose2d(31, 7, Math.toRadians(135)), Math.toRadians(135))
                        .waitSeconds(.75)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(58, 12, Math.toRadians(-180)), Math.toRadians(0))
                        .waitSeconds(.75)
                        .setReversed(false)
                        //3rd cone
                        .splineToLinearHeading(new Pose2d(31, 7, Math.toRadians(135)), Math.toRadians(135))
                        .waitSeconds(.75)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(58, 12, Math.toRadians(-180)), Math.toRadians(0))
                        .waitSeconds(.75)
                        .setReversed(false)
                        //4th cone
                        .splineToLinearHeading(new Pose2d(31, 7, Math.toRadians(135)), Math.toRadians(135))
                        .waitSeconds(.75)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(58, 12, Math.toRadians(-180)), Math.toRadians(0))
                        .waitSeconds(.75)
                        .setReversed(false)
                        //5th cone
                        .splineToLinearHeading(new Pose2d(31, 7, Math.toRadians(135)), Math.toRadians(135))
                        .waitSeconds(.75)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(58, 12, Math.toRadians(-180)), Math.toRadians(0))
                        .waitSeconds(.75)
                        .setReversed(false)
                        //6th cone
                        .splineToLinearHeading(new Pose2d(31, 7, Math.toRadians(135)), Math.toRadians(135))
                        //more ben test
                        /*
                        .addDisplacementMarker(() -> {
                            //ADD algorithm to make robot go to parking spot based off of parkingSpot int
                            //if (parkingSpot == 1){


                        })
                         */
                        .build()
        );
    }
}
