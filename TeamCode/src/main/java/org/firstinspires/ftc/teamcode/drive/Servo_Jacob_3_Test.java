package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Jacob Servo 3 Test")
@Disabled
public class Servo_Jacob_3_Test extends LinearOpMode {

    static final double SCALE = 0.01;
    static final int CYCLE_MS = 2;   // period of each cycle
    //static double INCREMENT_Y = 0.0;     // amount to slew servo each CYCLE_MS cycle
    //static double INCREMENT_X = 0.0;     // amount to slew servo each CYCLE_MS cycle
    static final double MAX_POS = 1.0;     // Maximum rotational position
    static final double MIN_POS = 0.0;     // Minimum rotational position

    //enum Direction {FORWARD, REVERSE}         // Define direction of servo

    // Define class members
    Servo liftOne; Servo liftTwo; Servo liftSpin;
    double position1 = 0.0; // Start position of liftOne servo
    double position2 = 0.0; // Start position of liftTwo servo
    double position3 = 0.0; // Start position of liftSpin servo


    @Override
    public void runOpMode() {

        // Change the text in quotes to match any servo name on your robot.
        liftOne = hardwareMap.get(Servo.class, "liftOne");
        liftOne.setDirection(Servo.Direction.FORWARD);
        liftTwo = hardwareMap.get(Servo.class, "liftTwo");
        liftTwo.setDirection(Servo.Direction.REVERSE);
        liftSpin = hardwareMap.get(Servo.class, "liftSpin");
        liftSpin.setDirection(Servo.Direction.FORWARD);

        // Wait for the start button
        telemetry.addData(">", "Press Start to move Lift/Spin Servos with Left Joystick.");
        telemetry.update();
        waitForStart();


        while (opModeIsActive()) {

            // slew the lift servos, according to the joystick y value until reaching min/max position.
            if (gamepad1.left_stick_y != 0) {
                //INCREMENT_Y = gamepad1.left_stick_y * SCALE;
                position1 += gamepad1.left_stick_y * SCALE;
                position2 += gamepad1.left_stick_y * SCALE;

                if (position1 >= MAX_POS) { position1 = MAX_POS; }
                if (position2 >= MAX_POS) { position2 = MAX_POS; }
                if (position1 <= MIN_POS) { position1 = MIN_POS; }
                if (position2 <= MIN_POS) { position2 = MIN_POS; }
            }
            // slew the spin servos, according to the joystick x value until reaching min/max position.
            if (gamepad1.left_stick_x != 0) {
                //INCREMENT_X = gamepad1.left_stick_x * SCALE;
                position3 += gamepad1.left_stick_x * SCALE;

                if (position3 >= MAX_POS) {
                    position3 = MAX_POS;
                }
                if (position3 <= MIN_POS) {
                    position3 = MIN_POS;
                }
            }

            // Display the current value
            telemetry.addData("liftOne Servo Position", "%5.2f", position1);
            telemetry.addData("liftTwo Servo Position", "%5.2f", position2);
            telemetry.addData("liftSpin Servo Position", "%5.2f", position3);
            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();

            // Set the servos to the new positions;
            liftOne.setPosition(position1);
            liftTwo.setPosition(position2);
            liftSpin.setPosition(position3);
            sleep(CYCLE_MS);
            idle();

        }
    }
}