/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Jacob Servo")

public class ServoJacob extends LinearOpMode {


    static final double SCALE       = 0.01;
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position


    // Define class members
    Servo  liftOne;
    Servo  liftTwo;
    Servo  liftSpin;
    double  position1 = 0.0;
    double  position2 = 0.0;
    double  position3 = 0.0;
   // boolean rampUp = true;


    @Override
    public void runOpMode() {


        // Change the text in quotes to match any servo name on your robot.
        liftOne = hardwareMap.get(Servo.class, "liftOne");
        liftTwo = hardwareMap.get(Servo.class, "liftTwo");
        liftSpin = hardwareMap.get(Servo.class, "liftSpin");

        liftOne.setDirection(Servo.Direction.FORWARD);
        liftTwo.setDirection(Servo.Direction.REVERSE);
        liftSpin.setDirection(Servo.Direction.FORWARD);

        // Wait for the start button
        telemetry.addData(">", "Press Start to move Servos with left joystick." );
        telemetry.update();
        waitForStart();


        while(opModeIsActive()){
            if (gamepad1.left_stick_y != 0) {
                position1 += gamepad1.left_stick_y * SCALE;
                position2 += gamepad1.left_stick_y * SCALE;
                if (position1 >= MAX_POS) {
                    position1 = MAX_POS;
                }
                if (position1 <= MIN_POS) {
                    position1 = MIN_POS;
                }
                if (position2 >= MAX_POS) {
                    position2 = MAX_POS;
                }
                if (position2 <= MIN_POS) {
                    position2 = MIN_POS;
                }
            }
            if (gamepad1.left_stick_x != 0) {
                position3 += gamepad1.left_stick_x * SCALE;
                if (position3 >= MAX_POS) {
                    position3 = MAX_POS;
                }
                if (position3 <= MIN_POS) {
                    position3 = MIN_POS;
                }
            }
            if (gamepad1.x) {
               position1 = 0;
               position2 = 0;
               position3 = 0;

            }
            if (gamepad1.y) {
                position1 = 1;
                position2 = 1;
                position3 = 1;
            }

            // Display the current value
            telemetry.addData("liftOne Servo Position", "%5.2f", position1);
            telemetry.addData("liftTwo Servo Position", "%5.2f", position2);
            telemetry.addData("liftSpin Servo Position", "%5.2f", position3);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the servo to the new position and pause;
            liftOne.setPosition(position1);
            liftTwo.setPosition(position2);
            liftSpin.setPosition(position3);
            //sleep(CYCLE_MS);
            idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
