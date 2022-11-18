package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
@TeleOp(name = "Slide Motors")

public class SlideMotors extends LinearOpMode {

    DcMotor slideOne;
    DcMotor slideTwo;

    int desiredPos = 0;
    int runToPos = 0;

    public static final double ARM_POWER    =  0.50 ;

    public void runOpMode() {
        slideOne = hardwareMap.get(DcMotor.class, "slideOne");
        slideTwo = hardwareMap.get(DcMotor.class, "slideTwo");

        slideOne.setDirection(DcMotor.Direction.FORWARD);
        slideTwo.setDirection(DcMotor.Direction.REVERSE);
        slideOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData(">", "Robot Ready.  Press Play.");    //

        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("current Pos: ", slideOne.getCurrentPosition());
            telemetry.addData("desiredPos: ", desiredPos);
            if (gamepad1.dpad_up) {
                desiredPos = 1612;
                slideOne.setTargetPosition(desiredPos);
                slideTwo.setTargetPosition(desiredPos);
                slideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideOne.setPower(ARM_POWER);
                slideTwo.setPower(ARM_POWER);
                while((slideOne.isBusy())&&(slideTwo.isBusy())){}
                slideOne.setPower(0);
                slideTwo.setPower(0);
            }
            else if (gamepad1.dpad_left) {
                desiredPos = 1075;
                slideOne.setTargetPosition(desiredPos);
                slideTwo.setTargetPosition(desiredPos);
                slideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideOne.setPower(ARM_POWER);
                slideTwo.setPower(ARM_POWER);
                while((slideOne.isBusy())&&(slideTwo.isBusy())){}
                slideOne.setPower(0);
                slideTwo.setPower(0);
            }
            else if (gamepad1.dpad_down){
                desiredPos = 0;
                slideOne.setTargetPosition(desiredPos);
                slideTwo.setTargetPosition(desiredPos);
                slideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideOne.setPower(ARM_POWER);
                slideTwo.setPower(ARM_POWER);
                while((slideOne.isBusy())&&(slideTwo.isBusy())){}
                slideOne.setPower(0);
                slideTwo.setPower(0);
            }
            telemetry.update();
        }
    }
}
