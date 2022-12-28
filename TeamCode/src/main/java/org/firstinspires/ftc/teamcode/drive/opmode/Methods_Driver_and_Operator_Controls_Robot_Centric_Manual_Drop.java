package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "1) Methods Main Driver Robot Centric (Manual Drop)")

public class Methods_Driver_and_Operator_Controls_Robot_Centric_Manual_Drop extends LinearOpMode {
    ArmControl armControl = new ArmControl(true, false, this);
    @Override
    public void runOpMode() throws InterruptedException {
        armControl.Init(hardwareMap);
        armControl.StartPosition();

        waitForStart();

        if (isStopRequested()) return;

        while(opModeIsActive()){
            armControl.driveControlsRobotCentric();

            if (gamepad1.left_bumper) {
                armControl.openClaw();
            }
            if (gamepad1.right_bumper) {
                armControl.closeClaw();
            }

            if ((((armControl.rangeClaw < 2.75) && (armControl.armGrip.getPosition() == .18)) && (armControl.slideOne.getCurrentPosition() <= 100) && (armControl.slideTwo.getCurrentPosition() <= 100))) {
                armControl.StowCone(null);
            }
            //----------------------------------------------------------------
            // AutoGrab
            //----------------------------------------------------------------
            //----------------------------------------------------------------
            // High - move to high junction position from stow
            //----------------------------------------------------------------
            if ((gamepad1.y) && (!armControl.intake)) {
                armControl.GoToHigh(null);
            }
            //----------------------------------------------------------------
            // Medium - move to medium junction position from stow
            //----------------------------------------------------------------
            if ((gamepad1.x) && (!armControl.intake)){
                armControl.GoToMedium(null);
            }
            //----------------------------------------------------------------
            // Low - move to low junction position from stow
            //----------------------------------------------------------------
            if ((gamepad1.a) && (!armControl.intake)){
                armControl.GoToLow(null);
            }

            //----------------------------------------------------------------
            // ground pos - reset to cone intake position from high, medium, and low positions only
            //----------------------------------------------------------------
            if ((gamepad1.b)&&(!(armControl.stow))&&((armControl.slideOne.getCurrentPosition()>5)||(armControl.slideTwo.getCurrentPosition()>5))){
                armControl.ready = false;
                if (!armControl.high) {
                    armControl.ReturnFromLowMedium(null);
                }
                else if(armControl.high){
                    armControl.ReturnFromHigh(null);
                }
            }
            // Display the current value
            telemetry.addData("spinOne Servo Position", "%5.2f", armControl.spinOne.getPosition());
            telemetry.addData("spinTwo Servo Position", "%5.2f", armControl.spinTwo.getPosition());
            telemetry.addData("armRote Servo Position", "%5.2f", armControl.armRote.getPosition());
            telemetry.addData("liftWrist Servo Position", "%5.2f", armControl.liftWrist.getPosition());
            telemetry.addData("armGrip Servo Position", "%5.2f", armControl.armGrip.getPosition());
            telemetry.addData("slideOne motor Position", armControl.slideOne.getCurrentPosition());
            telemetry.addData("slideTwo motor Position", armControl.slideTwo.getCurrentPosition());
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();
        }
    }
}
