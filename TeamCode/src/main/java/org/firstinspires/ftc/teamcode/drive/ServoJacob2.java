package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Jacob Servo 2")

public class ServoJacob2 extends LinearOpMode {

    static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    //   static final int    CYCLE_MS    =   50;   // period of each cycle
    static final double MAX_POS = 1.0;     // Maximum rotational position
    static final double MIN_POS = 0.0;     // Minimum rotational position

    enum Direction {FORWARD, REVERSE}         // Define direction of servo

    // Define class members
    Servo liftOne;
    Servo liftTwo;
    Servo liftSpin;
    double position1 = 0.0;
    double position2 = 0.0; // Servo start position
    double position3 = 0.0;
    // boolean rampUp = true;


    @Override
    public void runOpMode() {


        // Change the text in quotes to match any servo name on your robot.
        liftOne = hardwareMap.get(Servo.class, "liftOne");


        liftOne.setDirection(Servo.Direction.FORWARD);


        // Wait for the start button
        telemetry.addData(">", "Press Start to move lifeOne Servo with left_stick_y joystick.");
        telemetry.update();
        waitForStart();


        // Scan servo till stop pressed.
        while (opModeIsActive()) {

            // slew the servo, according to the rampUp (direction) variable.
            if (gamepad1.left_stick_y < 0) {
                // Keep stepping up until we hit the max value.
                position1 += INCREMENT;
                if (position1 >= MAX_POS) {
                    position1 = MAX_POS;

                }
            } else if (gamepad1.left_stick_y > 0) {
                // Keep stepping down until we hit the minimum value.
                position1 -= INCREMENT;
                if (position1 <= MIN_POS) {
                    position1 = MIN_POS;

                }
            }

            // Display the current value
            telemetry.addData("liftOne Servo Position", "%5.2f", position1);
            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();

            // Set the servo to the new position and pause;
            liftOne.setPosition(position1);
            //sleep(CYCLE_MS);
            idle();
        }
        // Servo Two

        liftTwo = hardwareMap.get(Servo.class, "liftTwo");
        liftTwo.setDirection(Servo.Direction.REVERSE);
        telemetry.addData(">", "Press Start to move lifeTwo Servo with left_stick_y joystick.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // slew the servo, according to the rampUp (direction) variable.
            if (gamepad1.left_stick_y < 0) {
                // Keep stepping up until we hit the max value.
                position2 += INCREMENT;
                if (position2 >= MAX_POS) {
                    position2 = MAX_POS;

                }
            } else if (gamepad1.left_stick_y > 0) {
                // Keep stepping down until we hit the minimum value.
                position2 -= INCREMENT;
                if (position2 <= MIN_POS) {
                    position2 = MIN_POS;

                }
            }

            // Display the current value
            telemetry.addData("liftTwo Servo Position", "%5.2f", position2);
            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();

            // Set the servo to the new position and pause;
            liftTwo.setPosition(position2);
            //sleep(CYCLE_MS);
            idle();
            // Servo Three

            liftSpin = hardwareMap.get(Servo.class, "liftSpin");
            liftSpin.setDirection(Servo.Direction.FORWARD);
            telemetry.addData(">", "Press Start to move lifeSpin Servo with left_stick_x joystick.");
            telemetry.update();
            waitForStart();

            while (opModeIsActive()) {

                // slew the servo, according to the rampUp (direction) variable.
                if (gamepad1.left_stick_x < 0) {
                    // Keep stepping up until we hit the max value.
                    position3 += INCREMENT;
                    if (position3 >= MAX_POS) {
                        position3 = MAX_POS;

                    }
                } else if (gamepad1.left_stick_x > 0) {
                    // Keep stepping down until we hit the minimum value.
                    position3 -= INCREMENT;
                    if (position3 <= MIN_POS) {
                        position3 = MIN_POS;

                    }
                }

                // Display the current value
                telemetry.addData("liftSpin Servo Position", "%5.2f", position3);
                telemetry.addData(">", "Press Stop to end test.");
                telemetry.update();

                // Set the servo to the new position and pause;
                liftSpin.setPosition(position3);
                //sleep(CYCLE_MS);
                idle();

                // Signal done;
                telemetry.addData(">", "Done");
                telemetry.update();
            }
        }
    }
}