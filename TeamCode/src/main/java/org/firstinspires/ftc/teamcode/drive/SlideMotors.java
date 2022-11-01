package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Slide Motors")

public class SlideMotors extends LinearOpMode {

    DcMotor slideOne;
    DcMotor slideTwo;


    public static final double ARM_UP_POWER    =  0.50 ;
    public static final double ARM_DOWN_POWER  = 0.50 ;

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
                slideOne.setPower(ARM_UP_POWER);
                slideTwo.setPower(ARM_UP_POWER);
            }else if (gamepad1.dpad_down) {
                slideOne.setPower(ARM_DOWN_POWER);
                slideTwo.setPower(ARM_DOWN_POWER);
            }else {
                slideOne.setPower(0.0);
                slideTwo.setPower(0.0);
            }
        }
    }
}
