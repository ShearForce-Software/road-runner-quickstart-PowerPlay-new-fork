package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Jacob Slide Drive using Encoder")
//@Disabled
public class SlideMotorJacob extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor leftSlide = null;
    private DcMotor rightSlide = null;

    static final double COUNTS_PER_MOTOR_REV = 538;  // eg: AndyMark 19.2 Motor Encoder
    static final double WHEEL_DIAMETER_INCHES = 1.0;  // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.14159);
    static final double DRIVE_SPEED = 0.5;
    static final double INTAKE = 0.0;
    static final double HIGH_JUNCTION = 21.5;
    static final double MEDIUM_JUNCTION = 14.5;
    static final double LOW_JUNCTION = 7.5;
    static final double GROUND_JUNCTION = 0.5;
    int leftTarget = 0;
    int rightTarget = 0;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables
        leftSlide = hardwareMap.get(DcMotor.class, "left_slide_drive");
        rightSlide = hardwareMap.get(DcMotor.class, "right_slide_drive");

        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setDirection(DcMotor.Direction.FORWARD);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d",
                leftSlide.getCurrentPosition(), rightSlide.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)

        waitForStart();

        while (opModeIsActive()) {

            // Use DPad to raise and lower lift manually
            if (gamepad1.dpad_up) {
                leftSlide.setPower(DRIVE_SPEED);
                rightSlide.setPower(DRIVE_SPEED);
            }
            else if (gamepad1.dpad_down) {
                leftSlide.setPower(-DRIVE_SPEED);
                rightSlide.setPower(-DRIVE_SPEED);
            }
            else {
                leftSlide.setPower(0);
                rightSlide.setPower(0);
            }

            telemetry.addData("Encoder Positions: ", "%7d :%7d",
                    leftSlide.getCurrentPosition(), rightSlide.getCurrentPosition());
            telemetry.update();

            // Use X Button to automatically return lift Intake starting height
            if (gamepad1.x) {
                // Calculate INTAKE POSITION motor ticks
                leftTarget = (int) (INTAKE * COUNTS_PER_INCH);
                rightTarget = (int) (INTAKE * COUNTS_PER_INCH);

                // Set Encoder Target
                leftSlide.setTargetPosition(leftTarget);
                rightSlide.setTargetPosition(rightTarget);

                // Turn On RUN_TO_POSITION
                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // Set Power to start motion
                leftSlide.setPower(DRIVE_SPEED);
                rightSlide.setPower(DRIVE_SPEED);

                while (leftSlide.isBusy() || rightSlide.isBusy()) {
                    telemetry.addData("Running to", " %7d :%7d", leftTarget, rightTarget);
                    telemetry.addData("Currently at", " at %7d :%7d",
                            leftSlide.getCurrentPosition(), rightSlide.getCurrentPosition());
                    telemetry.update();
                    idle();
                }

                // Stop all motion;
                leftSlide.setPower(0);
                rightSlide.setPower(0);

                // Turn off RUN_TO_POSITION
                leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }
            // Use Y Button to automatically return lift to High Junction delivery height
            else if (gamepad1.y) {
                // Calculate INTAKE POSITION motor ticks
                leftTarget = (int) (HIGH_JUNCTION * COUNTS_PER_INCH);
                rightTarget = (int) (HIGH_JUNCTION * COUNTS_PER_INCH);

                // Set Encoder Target
                leftSlide.setTargetPosition(leftTarget);
                rightSlide.setTargetPosition(rightTarget);

                // Turn On RUN_TO_POSITION
                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // Set Power to start motion
                leftSlide.setPower(DRIVE_SPEED);
                rightSlide.setPower(DRIVE_SPEED);

                while (leftSlide.isBusy() || rightSlide.isBusy()) {
                    telemetry.addData("Running to", " %7d :%7d", leftTarget, rightTarget);
                    telemetry.addData("Currently at", " at %7d :%7d",
                            leftSlide.getCurrentPosition(), rightSlide.getCurrentPosition());
                    telemetry.update();
                    idle();
                }

                // Stop all motion;
                leftSlide.setPower(0);
                rightSlide.setPower(0);

                // Turn off RUN_TO_POSITION
                leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }
            // Use B Button to automatically return lift to Medium Junction delivery height
            else if (gamepad1.b) {
                // Calculate INTAKE POSITION motor ticks
                leftTarget = (int) (MEDIUM_JUNCTION * COUNTS_PER_INCH);
                rightTarget = (int) (MEDIUM_JUNCTION * COUNTS_PER_INCH);

                // Set Encoder Target
                leftSlide.setTargetPosition(leftTarget);
                rightSlide.setTargetPosition(rightTarget);

                // Turn On RUN_TO_POSITION
                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // Set Power to start motion
                leftSlide.setPower(DRIVE_SPEED);
                rightSlide.setPower(DRIVE_SPEED);

                while (leftSlide.isBusy() || rightSlide.isBusy()) {
                    telemetry.addData("Running to", " %7d :%7d", leftTarget, rightTarget);
                    telemetry.addData("Currently at", " at %7d :%7d",
                            leftSlide.getCurrentPosition(), rightSlide.getCurrentPosition());
                    telemetry.update();
                    idle();
                }

                // Stop all motion;
                leftSlide.setPower(0);
                rightSlide.setPower(0);

                // Turn off RUN_TO_POSITION
                leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }
            // Use A Button to automatically return lift to Low Junction delivery height
            else if (gamepad1.a) {
                // Calculate INTAKE POSITION motor ticks
                leftTarget = (int) (LOW_JUNCTION * COUNTS_PER_INCH);
                rightTarget = (int) (LOW_JUNCTION * COUNTS_PER_INCH);

                // Set Encoder Target
                leftSlide.setTargetPosition(leftTarget);
                rightSlide.setTargetPosition(rightTarget);

                // Turn On RUN_TO_POSITION
                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // Set Power to start motion
                leftSlide.setPower(DRIVE_SPEED);
                rightSlide.setPower(DRIVE_SPEED);

                while (leftSlide.isBusy() || rightSlide.isBusy()) {
                    telemetry.addData("Running to", " %7d :%7d", leftTarget, rightTarget);
                    telemetry.addData("Currently at", " at %7d :%7d",
                            leftSlide.getCurrentPosition(), rightSlide.getCurrentPosition());
                    telemetry.update();
                    idle();
                }

                // Stop all motion;
                leftSlide.setPower(0);
                rightSlide.setPower(0);

                // Turn off RUN_TO_POSITION
                leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }
}
