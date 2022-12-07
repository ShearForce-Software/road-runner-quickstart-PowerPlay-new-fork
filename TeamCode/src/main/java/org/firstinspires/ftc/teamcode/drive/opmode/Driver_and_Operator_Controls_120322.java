package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Main Driver Robot Centric")

public class Driver_and_Operator_Controls_120322 extends LinearOpMode {

    // Define class members
    Servo  spinOne;
    Servo  spinTwo;
    Servo  armRote;
    Servo  liftWrist;
    Servo  armGrip;
    DcMotor slideOne;
    DcMotor slideTwo;
    DcMotor leftFront;
    DcMotor leftRear;
    DcMotor rightFront;
    DcMotor rightRear;
    DistanceSensor rearDistance;
    DistanceSensor clawDistance;
    DistanceSensor frontDistance;

    double  position1 = 0.95;
    double  position2 = 0.95;
    double  position3 = 0.11;
    double  position4 = 0.6;
    double  position5 = 0.0;
    long  claw_time = 2000;
    long  wrist_time = 2000;
    long  spin_time = 2000;
    long  rotate_time = 2000;
    int START_POS = 5;
    int STOW_POS = 1400;
    int LOW_POS = 1850; //2090;
    int MED_POS = 3560; //3681;
    int HIGH_POS = 1550; //1738;
    boolean high = false;
    boolean movingDown = false;
    boolean stow = false;
    boolean ready = false;
    public static final double ARM_POWER    =  1 ;

    @Override
    public void runOpMode() throws InterruptedException {

        hardwareMap();
        motorInit();

        waitForStart();
        if (isStopRequested()) return;

        while(opModeIsActive()){
            driveControls(leftFront, leftRear, rightFront, rightRear);

            if (gamepad1.left_bumper) {
                position5 = .18;
            }
            if (gamepad1.right_bumper) {
                position5 = 0;
            }
            double rangeRear = rearDistance.getDistance(DistanceUnit.CM);
            double rangeClaw = clawDistance.getDistance(DistanceUnit.CM);
            double rangeFront = frontDistance.getDistance(DistanceUnit.CM);

            //----------------------------------------------------------------
            // AutoDrop
            //----------------------------------------------------------------
            if((rangeRear < 2.5)&&(ready)){
                //************************************************************
                // open claw when pole is detected by distance sensor
                //************************************************************
                position5 = .18;
            }
            //----------------------------------------------------------------
            // AutoGrab
            //----------------------------------------------------------------
            if ((((rangeClaw < 2.75) && (position5 == .18)) && (slideOne.getCurrentPosition() <= 100) && (slideTwo.getCurrentPosition() <= 100))) {

                //************************************************************
                // close claw when cone is detected by distance sensor
                //************************************************************
                position5 = 0;                  // armGrip close position
                armGrip.setPosition(position5); // close claw
                claw_time = 300; //200         // claw close time

                // wait for claw to close
                for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(claw_time); stop>System.nanoTime();) {
                    // check driver input while claw is moving
                    driveControls(leftFront, leftRear, rightFront, rightRear);}
                    //slideHeight();}
                //************************************************************
                // raise slide to cone stow position height
                //************************************************************
                slideOne.setTargetPosition(STOW_POS);
                slideTwo.setTargetPosition(STOW_POS);
                slideOne.setPower(ARM_POWER);
                slideTwo.setPower(ARM_POWER);
                while ((slideOne.isBusy()) && (slideTwo.isBusy())){
                    // check driver input while slides are moving
                    driveControls(leftFront, leftRear, rightFront, rightRear);}
                    //slideHeight();}
                slideOne.setPower(0);
                slideTwo.setPower(0);

                //************************************************************
                // straighten wrist before spinning arm
                //************************************************************
                position4 = .35;                    // liftWrist straight position
                liftWrist.setPosition(position4);   // straighten wrist
                wrist_time = 200; //200            // wrist straighten time

                // wait for wrist to straighten
                for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(wrist_time); stop>System.nanoTime();) {
                    // check driver input while servos are moving
                    driveControls(leftFront, leftRear, rightFront, rightRear);}
                    //slideHeight();}

                //************************************************************
                // spin arm to safe cone rotate position
                //************************************************************
                position1 = .72;                // spinOne safe cone rotate position
                position2 = .72;                // spinTwo safe cone rotate position
                spinOne.setPosition(position1); // spin arm to safe cone rotate position
                spinTwo.setPosition(position2); // spin arm to safe cone rotate position
                spin_time = 500; //500         // spin arm time

                // wait for arm to swing
                for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(spin_time); stop>System.nanoTime();) {
                    // check driver input while servos are moving
                    driveControls(leftFront, leftRear, rightFront, rightRear);}
                    //slideHeight();}

                //************************************************************
                // rotate arm 180 degrees to flip cone
                //************************************************************
                position3 = .82;                // armRote position
                armRote.setPosition(position3); // rotate arm 180 degrees
                rotate_time = 600; //400       // rotate arm time

                // wait for arm to rotate
                for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(rotate_time); stop>System.nanoTime();) {
                    // check driver input while servos are moving
                    driveControls(leftFront, leftRear, rightFront, rightRear);}
                    //slideHeight();}

                //************************************************************
                // wrist to delivery angle (low / Medium)
                //************************************************************
                position4 = .86;        // 0.77     // liftWrist delivery position
                liftWrist.setPosition(position4);   // wrist to delivery angle
                wrist_time = 0; //200            // wrist move time

                // wait for wrist to move to delivery angle
                for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(wrist_time); stop>System.nanoTime();) {
                    // check driver input while servos are moving
                    driveControls(leftFront, leftRear, rightFront, rightRear);}
                    //slideHeight();}

                //************************************************************
                // spin arm to safe stow position
                //************************************************************
                position1 = .92;                // spinOne safe stow position
                position2 = .92;                // spinTwo safe stow position
                spinOne.setPosition(position1); // spin arm to safe stow position
                spinTwo.setPosition(position2); // spin arm to safe stow position
                spin_time = 200; //200         // spin arm time

                // wait for arm to swing
                for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(spin_time); stop>System.nanoTime();) {
                    // check driver input while servos are moving
                    driveControls(leftFront, leftRear, rightFront, rightRear);}
                    //slideHeight();}
                // set stow variable to true
                stow = true;
            }
            //----------------------------------------------------------------
            // High - move to high junction position from stow
            //----------------------------------------------------------------
            if ((gamepad1.y) && (stow)) {
                high = true;        // set position variable for return

                //************************************************************
                // verify claw is closed
                //************************************************************
                position5 = 0;
                armGrip.setPosition(position5);

                //************************************************************
                // raise slides to high junction delivery height
                //************************************************************
                slideOne.setTargetPosition(HIGH_POS);
                slideTwo.setTargetPosition(HIGH_POS);
                slideOne.setPower(ARM_POWER);
                slideTwo.setPower(ARM_POWER);
                while ((slideOne.isBusy()) && (slideTwo.isBusy())){
                    // check driver input while slides are moving
                    driveControls(leftFront, leftRear, rightFront, rightRear);}
                slideOne.setPower(0);
                slideTwo.setPower(0);

                //************************************************************
                // swinging arm to high junction position
                //************************************************************
                position1 = .11;
                position2 = .11;
                spinOne.setPosition(position1);
                spinTwo.setPosition(position2);
                spin_time = 400; //1000

                for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(spin_time); stop>System.nanoTime();) {
                    // check driver input while servos are moving
                    driveControls(leftFront, leftRear, rightFront, rightRear);}

                //************************************************************
                // wrist to delivery angle (high)
                //************************************************************
                position4 = .13;
                liftWrist.setPosition(position4);

                // set stow variable to false
                stow = false;
                ready = true;
            }

            //----------------------------------------------------------------
            // Medium - move to medium junction position from stow
            //----------------------------------------------------------------
            if ((gamepad1.x) && (stow)){
                high = false;       // set position variable for return

                //************************************************************
                // verify claw is closed
                //************************************************************
                position5 = 0;
                armGrip.setPosition(position5);

                //************************************************************
                // raise slides to medium junction delivery height
                //************************************************************
                slideOne.setTargetPosition(MED_POS);
                slideTwo.setTargetPosition(MED_POS);
                slideOne.setPower(ARM_POWER);
                slideTwo.setPower(ARM_POWER);
                while ((slideOne.isBusy()) && (slideTwo.isBusy())){
                    // check driver input while slides are moving
                    driveControls(leftFront, leftRear, rightFront, rightRear);}
                slideOne.setPower(0);
                slideTwo.setPower(0);

                //************************************************************
                // spin arm to medium junction position
                //************************************************************
                position1 = .83;
                position2 = .83;
                spinOne.setPosition(position1);
                spinTwo.setPosition(position2);
                spin_time = 0; //200

                for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(spin_time); stop>System.nanoTime();) {
                    // check driver input while servos are moving
                    driveControls(leftFront, leftRear, rightFront, rightRear);}

                //************************************************************
                // wrist to delivery angle (Medium)
                //************************************************************
                position4 = .86;                  // liftWrist delivery position
                liftWrist.setPosition(position4); // wrist to delivery angle
                wrist_time = 0; //200             // wrist move time

                // wait for wrist to move to delivery angle
                for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(wrist_time); stop>System.nanoTime();) {
                    // check driver input while servos are moving
                    driveControls(leftFront, leftRear, rightFront, rightRear);}

                // set stow variable to false
                stow = false;
                ready = true;
            }
            //----------------------------------------------------------------
            // Low - move to low junction position from stow
            //----------------------------------------------------------------
            if ((gamepad1.a) && (stow)){
                high = false;       // set position variable for return

                //************************************************************
                // verify claw is closed
                //************************************************************
                position5 = 0;
                armGrip.setPosition(position5);

                //************************************************************
                // raise slides to low junction delivery height
                //************************************************************
                slideOne.setTargetPosition(LOW_POS);
                slideTwo.setTargetPosition(LOW_POS);
                slideOne.setPower(ARM_POWER);
                slideTwo.setPower(ARM_POWER);
                while ((slideOne.isBusy()) && (slideTwo.isBusy())){
                    // check driver input while slides are moving
                    driveControls(leftFront, leftRear, rightFront, rightRear);}
                slideOne.setPower(0);
                slideTwo.setPower(0);

                //************************************************************
                // spin arm to low junction position
                //************************************************************
                position1 = .83;
                position2 = .83;
                spinOne.setPosition(position1);
                spinTwo.setPosition(position2);
                spin_time = 0; //200

                for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(spin_time); stop>System.nanoTime();) {
                    // check driver input while servos are moving
                    driveControls(leftFront, leftRear, rightFront, rightRear);}

                //************************************************************
                // wrist to delivery angle (Low)
                //************************************************************
                position4 = .86;                  // liftWrist delivery position
                liftWrist.setPosition(position4); // wrist to delivery angle
                wrist_time = 0; //200             // wrist move time

                // wait for wrist to move to delivery angle
                for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(wrist_time); stop>System.nanoTime();) {
                    // check driver input while servos are moving
                    driveControls(leftFront, leftRear, rightFront, rightRear);}

                // set stow variable to false
                stow = false;
                ready = true;
            }

            //----------------------------------------------------------------
            // ground pos - reset to cone intake position from high, medium, and low positions only
            //----------------------------------------------------------------
            if ((gamepad1.b)&&(!(stow))&&((slideOne.getCurrentPosition()>5)||(slideTwo.getCurrentPosition()>5))){
                ready = false;
                if (!high) {
                    //************************************************************
                    // Close claw
                    //************************************************************
                    position5 = 0;
                    armGrip.setPosition(position5);
                    claw_time = 200;
                    for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(claw_time); stop>System.nanoTime();) {
                        // check driver input while servos are moving
                        driveControls(leftFront, leftRear, rightFront, rightRear);}

                    //************************************************************
                    // spin arm to safe return position
                    //************************************************************
                    position1 = .95;    //0.84
                    position2 = .95;    //0.84
                    spinOne.setPosition(position1);
                    spinTwo.setPosition(position2);
                    spin_time = 300;

                    for (long stop = System.nanoTime() + TimeUnit.MILLISECONDS.toNanos(spin_time); stop > System.nanoTime(); ) {
                        // check driver input while servos are moving
                        driveControls(leftFront, leftRear, rightFront, rightRear);
                        // check if claw sensor is detecting something is in the way
                        safetyCheck(rangeClaw);}

                    //************************************************************
                    // straighten wrist
                    //************************************************************
                    position4 = .35;
                    liftWrist.setPosition(position4);
                    wrist_time = 300;

                    for (long stop = System.nanoTime() + TimeUnit.MILLISECONDS.toNanos(wrist_time); stop > System.nanoTime(); ) {
                        driveControls(leftFront, leftRear, rightFront, rightRear);
                        safetyCheck(rangeClaw);}

                    //************************************************************
                    // rotate arm 180 degrees
                    //************************************************************
                    position3 = .11;
                    armRote.setPosition(position3);
                    rotate_time = 550;

                    for (long stop = System.nanoTime() + TimeUnit.MILLISECONDS.toNanos(rotate_time); stop > System.nanoTime(); ) {
                        driveControls(leftFront, leftRear, rightFront, rightRear);
                        safetyCheck(rangeClaw);}

                    //************************************************************
                    // wrist to cone pickup position
                    //************************************************************
                    position4 = .60;
                    liftWrist.setPosition(position4);
                    wrist_time = 200;

                    for (long stop = System.nanoTime() + TimeUnit.MILLISECONDS.toNanos(wrist_time); stop > System.nanoTime(); ) {
                        driveControls(leftFront, leftRear, rightFront, rightRear);
                        safetyCheck(rangeClaw);}

                    //************************************************************
                    // lower slides to cone intake height
                    //************************************************************
                    slideOne.setTargetPosition(START_POS);
                    slideTwo.setTargetPosition(START_POS);
                    slideOne.setPower(ARM_POWER);
                    slideTwo.setPower(ARM_POWER);
                    while ((slideOne.isBusy()) && (slideTwo.isBusy())) {
                        driveControls(leftFront, leftRear, rightFront, rightRear);
                        safetyCheck(rangeClaw);}
                    slideOne.setPower(0);
                    slideTwo.setPower(0);
                    movingDown = false;

                    //************************************************************
                    // Open claw
                    //************************************************************
                    position5 = .18;
                    armGrip.setPosition(position5);

                }
                else if(high){

                    //************************************************************
                    // Close claw
                    //************************************************************
                    position5 = 0;
                    armGrip.setPosition(position5);


                    //************************************************************
                    // rotate arm 180 degrees (so gripper is backwards)
                    //************************************************************
                    position3 = .11;
                    armRote.setPosition(position3);
                    rotate_time = 550;
                    for (long stop = System.nanoTime() + TimeUnit.MILLISECONDS.toNanos(rotate_time); stop > System.nanoTime(); ) {
                        driveControls(leftFront, leftRear, rightFront, rightRear);
                        safetyCheck(rangeClaw); }

                    //************************************************************
                    // spin arm to cone pickup position
                    //************************************************************
                    position1 = .95;
                    position2 = .95;
                    spinOne.setPosition(position1);
                    spinTwo.setPosition(position2);
                    spin_time = 700;

                    for (long stop = System.nanoTime() + TimeUnit.MILLISECONDS.toNanos(spin_time); stop > System.nanoTime(); ) {
                        driveControls(leftFront, leftRear, rightFront, rightRear);
                        safetyCheck(rangeClaw);}

                    //************************************************************
                    // wrist to cone pickup position
                    //************************************************************
                    position4 = .60;
                    liftWrist.setPosition(position4);
                    wrist_time = 450; //300

                    for (long stop = System.nanoTime() + TimeUnit.MILLISECONDS.toNanos(wrist_time); stop > System.nanoTime(); ) {
                        driveControls(leftFront, leftRear, rightFront, rightRear);
                        safetyCheck(rangeClaw);}

                    //************************************************************
                    // lower slides to cone intake height
                    //************************************************************
                    slideOne.setTargetPosition(START_POS);
                    slideTwo.setTargetPosition(START_POS);
                    slideOne.setPower(ARM_POWER);
                    slideTwo.setPower(ARM_POWER);
                    while ((slideOne.isBusy()) && (slideTwo.isBusy())) {
                        driveControls(leftFront, leftRear, rightFront, rightRear);
                        safetyCheck(rangeClaw);}
                    slideOne.setPower(0);
                    slideTwo.setPower(0);
                    movingDown = false;

                    //************************************************************
                    // Open claw
                    //************************************************************
                    position5 = .18;
                    armGrip.setPosition(position5);

                }
            }
            // Display the current value
            telemetry.addData("spinOne Servo Position", "%5.2f", spinOne.getPosition());
            telemetry.addData("spinTwo Servo Position", "%5.2f", spinTwo.getPosition());
            telemetry.addData("armRote Servo Position", "%5.2f", armRote.getPosition());
            telemetry.addData("liftWrist Servo Position", "%5.2f", liftWrist.getPosition());
            telemetry.addData("armGrip Servo Position", "%5.2f", armGrip.getPosition());
            telemetry.addData("slideOne motor Position", slideOne.getCurrentPosition());
            telemetry.addData("slideTwo motor Position", slideTwo.getCurrentPosition());
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the servos to the new positions
            spinOne.setPosition(position1);
            spinTwo.setPosition(position2);
            armRote.setPosition(position3);
            liftWrist.setPosition(position4);
            armGrip.setPosition(position5);
            idle();
        }
    }

//    private void slideHeight() {
//        if(gamepad1.a){
//            slideOne.setTargetPosition(LOW_POS);
//            slideTwo.setTargetPosition(LOW_POS);
//            slideOne.setPower(ARM_POWER);
//            slideTwo.setPower(ARM_POWER);
//            while ((slideOne.isBusy()) && (slideTwo.isBusy())){
//                // check driver input while slides are moving
//                driveControls(leftFront, leftRear, rightFront, rightRear);}
//            slideOne.setPower(0);
//            slideTwo.setPower(0);
//        }
//        if(gamepad1.x){
//            slideOne.setTargetPosition(MED_POS);
//            slideTwo.setTargetPosition(MED_POS);
//            slideOne.setPower(ARM_POWER);
//            slideTwo.setPower(ARM_POWER);
//            while ((slideOne.isBusy()) && (slideTwo.isBusy())){
//                // check driver input while slides are moving
//                driveControls(leftFront, leftRear, rightFront, rightRear);}
//            slideOne.setPower(0);
//            slideTwo.setPower(0);
//        }
//        if(gamepad1.y){
//            slideOne.setTargetPosition(HIGH_POS);
//            slideTwo.setTargetPosition(HIGH_POS);
//            slideOne.setPower(ARM_POWER);
//            slideTwo.setPower(ARM_POWER);
//            while ((slideOne.isBusy()) && (slideTwo.isBusy())){
//                // check driver input while slides are moving
//                driveControls(leftFront, leftRear, rightFront, rightRear);}
//            slideOne.setPower(0);
//            slideTwo.setPower(0);
//        }
//    }

    private void safetyCheck(double rangeClaw) {
        if (rangeClaw < 0)//a number)
        {
            spinOne.setPosition(spinOne.getPosition());
            spinTwo.setPosition(spinTwo.getPosition());
            armRote.setPosition(armRote.getPosition());
            liftWrist.setPosition(liftWrist.getPosition());
            armGrip.setPosition(armGrip.getPosition());
            slideOne.setPower(0);
            slideTwo.setPower(0);
        }
    }

    private void motorInit() {
        // Servo Directions
        spinOne.setDirection(Servo.Direction.FORWARD);
        spinTwo.setDirection(Servo.Direction.REVERSE);
        armRote.setDirection(Servo.Direction.FORWARD);
        liftWrist.setDirection(Servo.Direction.FORWARD);
        armGrip.setDirection(Servo.Direction.FORWARD);

        // Chassis Motor Config
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // Slide Motor Config
        slideOne.setDirection(DcMotor.Direction.FORWARD);
        slideTwo.setDirection(DcMotor.Direction.FORWARD);
        slideOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the servos to the start positions
        spinOne.setPosition(position1);
        spinTwo.setPosition(position2);
        armRote.setPosition(position3);
        liftWrist.setPosition(position4);
        armGrip.setPosition(position5);

        // Wait for the start button
        telemetry.addData(">", "Hardware ready.  Press Start to begin" );
        telemetry.update();
    }
    private void hardwareMap() {
        // Servo names in hardware map on Control or Expansion Hubs
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
