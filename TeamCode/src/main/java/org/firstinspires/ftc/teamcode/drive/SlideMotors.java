package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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
        telemetry.addData(">", "Robot Ready.  Press Play.");    //

        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                desiredPos = 1612;
                runToPos = desiredPos - slideOne.getCurrentPosition();
                slideOne.setTargetPosition(runToPos);
                slideTwo.setTargetPosition(runToPos);
                slideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideOne.setPower(ARM_POWER);
                slideTwo.setPower(ARM_POWER);
            }else if (gamepad1.dpad_left) {
                desiredPos = 1075;
                runToPos = desiredPos - slideOne.getCurrentPosition();
                slideOne.setTargetPosition(runToPos);
                slideTwo.setTargetPosition(runToPos);
                slideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideOne.setPower(ARM_POWER);
                slideTwo.setPower(ARM_POWER);
            }else if (gamepad1.dpad_down){
                desiredPos = 0;
                runToPos -= slideOne.getCurrentPosition();
                slideOne.setTargetPosition(runToPos);
                slideTwo.setTargetPosition(runToPos);
                slideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideOne.setPower(ARM_POWER);
                slideTwo.setPower(ARM_POWER);

            }


        }
    }
}
