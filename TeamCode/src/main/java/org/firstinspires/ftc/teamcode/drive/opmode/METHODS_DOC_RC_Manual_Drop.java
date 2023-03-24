package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "1) Methods Main Driver Robot Centric (Manual Drop)")

public class METHODS_DOC_RC_Manual_Drop extends LinearOpMode {
    ArmControl armControl = new ArmControl(true, false, this);
    @Override
    public void runOpMode() throws InterruptedException {
        armControl.Init(hardwareMap);
        armControl.StartPosition(null, true);

        waitForStart();

        if (isStopRequested()) return;

        while(opModeIsActive()){
            armControl.driveControlsRobotCentric();

            if(gamepad1.dpad_down){
                armControl.SafetyStow(null);
            }
            if(gamepad1.dpad_up){
                armControl.StartPosition(null, true);
            }

            //~~~~~NEW MANUAL ARM STUFF~~~~~~\
            if(gamepad1.left_stick_y!=0){
                if(gamepad1.left_stick_y>0) {
                    armControl.slideOne.setTargetPosition(armControl.slideOne.getCurrentPosition() - 100);
                    armControl.slideTwo.setTargetPosition(armControl.slideTwo.getCurrentPosition() - 100);
                    armControl.slideOne.setPower(1);
                    armControl.slideTwo.setPower(1);
                    armControl.WaitForSlides(null);
                    armControl.slideOne.setPower(0);
                    armControl.slideTwo.setPower(0);
                }
                if(gamepad1.left_stick_y<0) {
                    armControl.slideOne.setTargetPosition(armControl.slideOne.getCurrentPosition() + 100);
                    armControl.slideTwo.setTargetPosition(armControl.slideTwo.getCurrentPosition() + 100);
                    armControl.slideOne.setPower(1);
                    armControl.slideTwo.setPower(1);
                    armControl.WaitForSlides(null);
                    armControl.slideOne.setPower(0);
                    armControl.slideTwo.setPower(0);
                }
            }
            /*
            if(gamepad1.right_trigger == 1 && gamepad1.left_trigger == 0) {
                armControl.ManualSlideAdjust(true);
            }
            else if(gamepad1.left_trigger == 1 && gamepad1.right_trigger == 0){
                armControl.ManualSlideAdjust(false);
            }
            else{
                armControl.ManualSlideAdjust();
            }
            */
            //ends here

            if (gamepad1.left_bumper) {
                armControl.openClaw();
            }
            if (gamepad1.right_bumper) {
                armControl.closeClaw();
            }

            if ((armControl.rangeClaw < 3 && armControl.armGrip.getPosition() == .18 && armControl.slideOne.getCurrentPosition() <= 100 && armControl.slideTwo.getCurrentPosition() <= 100) || gamepad1.dpad_left) {
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
            if ((gamepad1.b)&&(!armControl.stow)&&(armControl.rangeClaw > 2.75)&&((armControl.slideOne.getCurrentPosition()>5)||(armControl.slideTwo.getCurrentPosition()>5))){
                armControl.readyToDrop = false;
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
