package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.TimeUnit;

@TeleOp(name="FSM Driver Control")
@Disabled
public class FSM_Driver_and_Operator_Controls extends OpMode {

    public enum ConeState {
        CONE_START, // Auto pickup cone
        CONE_CAPTURE_AND_STOW, // Select and position for Ground, Low, Medium, High position
        CONE_DELIVER, // Open claw to drop cone on junction
        CONE_HOME_ARM // Return arm to Start position
    };

    // The coneState variable is declared out here so its value persists between loop() calls
    ConeState coneState = ConeState.CONE_START;

    // Declare OpMode members
    private Servo spinOne;
    private Servo spinTwo;
    private Servo armRote;
    private Servo liftWrist;
    private Servo armGrip;
    private DcMotor slideOne;
    private DcMotor slideTwo;
    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightFront;
    private DcMotor rightRear;
    private DistanceSensor rearDistance;
    private DistanceSensor clawDistance;
    private DistanceSensor frontDistance;

    double  spinOne_pos = 0.95;
    double  spinTwo_pos = 0.95;
    double  armRote_pos = 0.13;
    double  liftWrist_pos = 0.6;
    double  armGrip_pos = 0.0;
    public static final double  ARM_POWER = 1;
    private ElapsedTime runtime = new ElapsedTime();

    final int LIFT_HOME = 5;     // the start encoder position for the lift
    final int LIFT_STOW = 1400;   // the stow encoder position for the lift
    final int LIFT_GROUND = 200;  // the ground encoder position for the lift (verify)
    final int LIFT_LOW = 2090;    // the low encoder position for the lift
    final int LIFT_MEDIUM = 3681; // the medium encoder position for the lift
    final int LIFT_HIGH = 1738;   // the high encoder position for the lift

    // Code to run ONCE when the driver hits INIT

    @Override
    public void init() {
        hardwareMap();
        motorInit();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    // Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY

    @Override
    public void init_loop() {}

    // Code to run ONCE when the driver hits PLAY

    @Override
    public void start() {
        runtime.reset();
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP

    @Override
    public void loop() {

        // Manual drive controls

        driveControls(leftFront, leftRear, rightFront, rightRear);
        if (gamepad1.left_bumper) { armGrip_pos = .18; } // left bumper open claw
        if (gamepad1.right_bumper) { armGrip_pos = 0; }  // right bumper close claw

        // Distance sensors
        double rangeRear = rearDistance.getDistance(DistanceUnit.CM);
        double rangeClaw = clawDistance.getDistance(DistanceUnit.CM);
        double rangeFront = frontDistance.getDistance(DistanceUnit.CM);

        // State Machine Cases
        switch (coneState) {

            case CONE_START:
                //***********************************************************
                // Claw Sensor Detects, Automatically Captures and Stows Cone
                //***********************************************************
                if ((((rangeClaw < 2.75) && (armGrip_pos == .18)) && (slideOne.getCurrentPosition() <= 100) && (slideTwo.getCurrentPosition() <= 100))) {
                    armGrip_pos = 0;
                    spinOne_pos = .80;
                    spinTwo_pos = .80;
                    armGrip.setPosition(armGrip_pos);
                    for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(200); stop>System.nanoTime();) {
                        driveControls(leftFront, leftRear, rightFront, rightRear);
                    }
                    slideOne.setTargetPosition(LIFT_STOW);
                    slideTwo.setTargetPosition(LIFT_STOW);
                    slideOne.setPower(ARM_POWER);
                    slideTwo.setPower(ARM_POWER);
                    while ((slideOne.isBusy()) && (slideTwo.isBusy())){
                        driveControls(leftFront, leftRear, rightFront, rightRear);
                    }
                    slideOne.setPower(0);
                    slideTwo.setPower(0);
                    spinOne.setPosition(spinOne_pos);
                    spinTwo.setPosition(spinTwo_pos);

                    // Cone in stow position
                    coneState = ConeState.CONE_CAPTURE_AND_STOW;
                }
                break;

            case CONE_CAPTURE_AND_STOW:

                //******************************************************************
                // Wait for Operator input for High, Medium, Low, or Ground Junction
                // Position cone for delivery on selected junction
                //******************************************************************

                // position to High Junction
                if (gamepad1.y) {
                    spinOne_pos = .11;
                    spinTwo_pos = .11;
                    armRote_pos = .83;
                    liftWrist_pos = .13;
                    armGrip_pos = 0;
                    armGrip.setPosition(armGrip_pos);
                    slideOne.setTargetPosition(LIFT_HIGH);
                    slideTwo.setTargetPosition(LIFT_HIGH);
                    slideOne.setPower(ARM_POWER);
                    slideTwo.setPower(ARM_POWER);
                    while ((slideOne.isBusy()) && (slideTwo.isBusy())){
                        driveControls(leftFront, leftRear, rightFront, rightRear);
                    }
                    slideOne.setPower(0);
                    slideTwo.setPower(0);
                    spinOne.setPosition(spinOne_pos);
                    spinTwo.setPosition(spinTwo_pos);
                    for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(1000); stop>System.nanoTime();) {
                        driveControls(leftFront, leftRear, rightFront, rightRear);
                    }
                    armRote.setPosition(armRote_pos);
                    liftWrist.setPosition(liftWrist_pos);

                    // cone positioned for high junction delivery
                    coneState = ConeState.CONE_DELIVER;
                }

                // position to Medium Junction
                else if (gamepad1.x){
                    spinOne_pos = .48;
                    spinTwo_pos = .48;
                    armRote_pos = .83;
                    liftWrist_pos = .82;
                    armGrip_pos = 0;
                    armGrip.setPosition(armGrip_pos);
                    slideOne.setTargetPosition(LIFT_MEDIUM);
                    slideTwo.setTargetPosition(LIFT_MEDIUM);
                    slideOne.setPower(ARM_POWER);
                    slideTwo.setPower(ARM_POWER);
                    slideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while ((slideOne.isBusy()) && (slideTwo.isBusy())){
                        driveControls(leftFront, leftRear, rightFront, rightRear);
                    }
                    slideOne.setPower(0);
                    slideTwo.setPower(0);
                    spinOne.setPosition(spinOne_pos);
                    spinTwo.setPosition(spinTwo_pos);
                    for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(300); stop>System.nanoTime();) {
                        driveControls(leftFront, leftRear, rightFront, rightRear);
                    }
                    armRote.setPosition(armRote_pos);
                    liftWrist.setPosition(liftWrist_pos);
                    for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(400); stop>System.nanoTime();) {
                        driveControls(leftFront, leftRear, rightFront, rightRear);
                    }
                    spinOne_pos = .83;
                    spinTwo_pos = .83;
                    spinOne.setPosition(spinOne_pos);
                    spinTwo.setPosition(spinTwo_pos);

                    // cone positioned for medium junction delivery
                    coneState = ConeState.CONE_DELIVER;
                }
                // position to Low Junction
                else if (gamepad2.a){
                    spinOne_pos = .48;
                    spinTwo_pos = .48;
                    armRote_pos = .83;
                    liftWrist_pos = .82;
                    armGrip_pos = 0;
                    armGrip.setPosition(armGrip_pos);
                    slideOne.setTargetPosition(LIFT_LOW);
                    slideTwo.setTargetPosition(LIFT_LOW);
                    slideOne.setPower(ARM_POWER);
                    slideTwo.setPower(ARM_POWER);
                    while ((slideOne.isBusy()) && (slideTwo.isBusy())){
                        driveControls(leftFront, leftRear, rightFront, rightRear);
                    }
                    slideOne.setPower(0);
                    slideTwo.setPower(0);
                    spinOne.setPosition(spinOne_pos);
                    spinTwo.setPosition(spinTwo_pos);
                    for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(500); stop>System.nanoTime();) {
                        driveControls(leftFront, leftRear, rightFront, rightRear);
                    }
                    armRote.setPosition(armRote_pos);
                    liftWrist.setPosition(liftWrist_pos);
                    for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(400); stop>System.nanoTime();) {
                        driveControls(leftFront, leftRear, rightFront, rightRear);
                    }
                    spinOne_pos = .83;
                    spinTwo_pos = .83;
                    spinOne.setPosition(spinOne_pos);
                    spinTwo.setPosition(spinTwo_pos);

                    // cone positioned for low junction delivery
                    coneState = ConeState.CONE_DELIVER;
                }

                // position to Ground Junction
                else if (gamepad1.right_stick_button){
                    // add Ground Junction code here...
                }
                break;

            case CONE_DELIVER:

                //********************************************
                // Operator open claw to drop cone on junction
                //********************************************

                if (gamepad1.left_bumper){
                    armGrip_pos = 0.18;
                    armGrip.setPosition(armGrip_pos);
                    coneState = ConeState.CONE_HOME_ARM;
                }
                break;

            case CONE_HOME_ARM:

                //*****************************************************************
                // Operator press B to return arm to Start position to capture Cone
                //*****************************************************************
                if (gamepad1.b) {
                    spinOne_pos = 0.48;  // pos 1
                    spinTwo_pos = 0.48;  // pos 2
                    armRote_pos = 0.13;  // pos 3
                    liftWrist_pos = 0.74;// pos 4
                    armGrip_pos = 0.0;   // pos 5
                    armGrip.setPosition(armGrip_pos);
                    spinOne.setPosition(spinOne_pos);
                    spinTwo.setPosition(spinTwo_pos);
                    for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(500); stop>System.nanoTime();) {
                        driveControls(leftFront, leftRear, rightFront, rightRear);
                    }
                    armGrip.setPosition(armGrip_pos);
                    liftWrist.setPosition(liftWrist_pos);
                    armRote.setPosition(armRote_pos);
                    spinOne_pos = .95;
                    spinTwo_pos = .95;
                    for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(700); stop>System.nanoTime();) {
                        driveControls(leftFront, leftRear, rightFront, rightRear);
                    }
                    spinOne.setPosition(spinOne_pos);
                    spinTwo.setPosition(spinTwo_pos);
                    slideOne.setTargetPosition(LIFT_HOME);
                    slideTwo.setTargetPosition(LIFT_HOME);
                    slideOne.setPower(ARM_POWER);
                    slideTwo.setPower(ARM_POWER);
                    while ((slideOne.isBusy()) && (slideTwo.isBusy())){
                        driveControls(leftFront, leftRear, rightFront, rightRear);
                    }
                    slideOne.setPower(0);
                    slideTwo.setPower(0);
                    liftWrist_pos = .6;
                    armGrip_pos = .18;
                    liftWrist.setPosition(liftWrist_pos);
                    armGrip.setPosition(armGrip_pos);

                    // Move to Cone Capture State after reaching Home position
                    coneState = ConeState.CONE_START;
                }
                break;

            default:
                // should never be reached, as liftState should never be null
                coneState = ConeState.CONE_START;
        }

        // Display the current value
        telemetry.addData("spinOne Servo Position", "%5.2f", spinOne_pos);
        telemetry.addData("spinTwo Servo Position", "%5.2f", spinTwo_pos);
        telemetry.addData("armRote Servo Position", "%5.2f", armRote_pos);
        telemetry.addData("liftWrist Servo Position", "%5.2f", liftWrist_pos);
        telemetry.addData("armGrip Servo Position", "%5.2f", armGrip_pos);
        telemetry.addData("slideOne motor Position", slideOne.getCurrentPosition());
        telemetry.addData("slideTwo motor Position", slideTwo.getCurrentPosition());
        telemetry.addData(">", "Press Stop to end test." );
        telemetry.update();

        // Set the servos to the new positions
        spinOne.setPosition(spinOne_pos);
        spinTwo.setPosition(spinTwo_pos);
        armRote.setPosition(armRote_pos);
        liftWrist.setPosition(liftWrist_pos);
        armGrip.setPosition(armGrip_pos);
    }

    // Code to run ONCE after the driver hits STOP

    @Override
    public void stop() {}

    private void hardwareMap() {
        // Motor, Servo, Distance Sensor names in hardware map on Control or Expansion Hubs
        rearDistance = hardwareMap.get(DistanceSensor.class, "rearDistance");
        clawDistance = hardwareMap.get(DistanceSensor.class, "clawDistance");
        frontDistance = hardwareMap.get(DistanceSensor.class, "frontDistance");

        spinOne = hardwareMap.get(Servo.class, "spinOne");
        spinTwo = hardwareMap.get(Servo.class, "spinTwo");
        armRote = hardwareMap.get(Servo.class, "armRote");
        liftWrist = hardwareMap.get(Servo.class, "liftWrist");
        armGrip = hardwareMap.get(Servo.class, "armGrip");

        slideOne = hardwareMap.get(DcMotor.class, "slideOne");
        slideTwo = hardwareMap.get(DcMotor.class, "slideTwo");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightRear = hardwareMap.dcMotor.get("rightRear");
    }

    private void motorInit() {
        // Directions
        spinOne.setDirection(Servo.Direction.FORWARD);
        spinTwo.setDirection(Servo.Direction.REVERSE);
        armRote.setDirection(Servo.Direction.FORWARD);
        liftWrist.setDirection(Servo.Direction.FORWARD);
        armGrip.setDirection(Servo.Direction.FORWARD);
        slideOne.setDirection(DcMotor.Direction.FORWARD);
        slideTwo.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        slideOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void driveControls(DcMotor leftFront, DcMotor leftRear, DcMotor rightFront, DcMotor rightRear) {
        double y = gamepad2.left_stick_y;
        double x = -gamepad2.left_stick_x * 1.1;
        double rx = gamepad2.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);
    }
}
