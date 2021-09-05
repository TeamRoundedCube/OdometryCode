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
 *//*




*/
/*
      G O A L
 |A1|A2}A3|A4|A5|A6|
 |B1|B2}B3|B4|B5|B6|
 |C1|C2}C3|C4|C5|C6|
 |D1|D2}D3|D4|D5|D6|
 |E1|E2}E3|E4|E5|E6|
 |F1|F2}F3|F4|F5|F6|
     S T A R T

 *//*

// testing new branch
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name="DecemberAuto", group="FullBot")

public class AutoIntegratedByGyro extends LinearOpMode {

    */
/* Declare OpMode members. *//*

    public enum BlueDestination {
        A1, B2, C1
    }

    public enum RedDestination {
        A6, B5, C6
    }
//Making variables for future use
    public int appIdentifier;
    public double OBJECT_HEIGHT_LIMIT = 300;
    public float WOBBLE_BASE_2_ARMSENSOR_DISTANCE_MIN = 18.0f;
    public int WOBBLE_BLUE_COLOR_MIN = 200;
    private static final double ARM_DROP_OFF_ANGLE = 50.0f;
    private static final double ARM_RETRACT_ANGLE = 92.0f;
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "1Ring";
    private static final String LABEL_SECOND_ELEMENT = "4Ring";
    private static final String VUFORIA_KEY =
            "AUtexuD/////AAABmQIASsSYI0K7oKlLQXSWAklWJXXAgCte5uORbUIju1rDuc89XOps0SeZL95/obR5ocozZUTPbARiP/Jr6tkoyPVWNpQFZPCgwYoSmBv/wwAgOq21dkfd/BF8iR4diFLwOyyc9OY//6yF4nSM1qttUf+g576o/IHb+DLzMAzcmYwINNhBqIWxpuGDs7mDT0hOAllHo3kFRAeUf3pRl/1uzVOkUfD16IGhPy/ZSqHIiAvEgtZiGU+JC3h32hLDclTJka3KTb4Fo6fyUMpotamPiDA19zT+jdcNg8LJSDtQlymeDXVPGJs5rRwrQ/RPfKacFare/F+jrC7GWiS/jxDok4mJ5HIoRwMnKUeJTKl5ApCW";
    private static final int OBJECT_DETECT_WAIT = 5;
    private static final int ARM_MOTOR_WAIT = 5;
    private static final int CLAW_WAIT = 5;
    HardwareFullBot robot = new HardwareFullBot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    static final float TILE = 20.8f; // Each tile side
    static final double COUNTS_PER_MOTOR_REV = 537;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.93;     // For figuring circumference
    //static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
     //       (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double COUNTS_PER_INCH = 72.0;
    static final double ARM_COUNTS_PER_REV = 537;
    static final double ARM_COUNTS_PER_DEGREE = ARM_COUNTS_PER_REV / 360;
    static final double ARM_SPEED = 0.1;
    static final double DRIVE_SPEED = 0.5;
    static final double LEFT_FRONT_COEFF = 1.0;
    static final double LEFT_BACK_COEFF = 1.0;
    static final double RIGHT_FRONT_COEFF = 1.0;
    static final double RIGHT_BACK_COEFF = 1.0;

    static final double TURN_SPEED = 0.5;
    private VuforiaLocalizer vuforia;

    */
/**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     *//*

    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {

        */
/*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         *//*

        robot.init(hardwareMap);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        robot.claw.setPosition(1);
        sleep(2000);
      //  turnOnFlicker(0.5);
        telemetry.addData("running", "running");
        telemetry.update();

        waitForStart();

        //Starting position F2 and when code is ran, the destination will show and shows it's running.
        String startPosition = "F2";// A6,A5,A4,A3,A2,A1
        if (opModeIsActive()) {
          setShootingPosition();
          shootGoal();

            BlueDestination destination = getDestination(startPosition);
            telemetry.addData("destination", destination);
            telemetry.addData("running", "running");
            telemetry.update();

           gotoDestination(startPosition, destination); // Drive to Wobble Goal Delivery area           return A , B,C drive to square
           dropOffWobbleGoal(); // Lower Arm open claw
          //  comeBackToPickWobble(destination);

           gotoShootingLine(destination); // Go to shooting spot
           // shootAtGoal(destination); //Shoot the ring
           // driveForward(DRIVE_SPEED, 1f * TILE);
        }

        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
    private void comeBackToPickWobble(BlueDestination Destination) {
        turnRightTime(1.0, 1500);
        switch (Destination) {
            case A1:
                slideRight(11.0f);
                driveForward(1.0, 2.0 * TILE);

                break;
            case B2:
                slideRight(TILE);
                driveForward(1.0, 2.5 * TILE);

                break;
            case C1:
                driveForward(1.0, 1.5 * TILE);
                //strafeLeft(1, 1.0 * TILE, 10);
                break;
        }
    }
       private void findBlueWobble()
    {

    }

//Case statement added at specific time depending on the location.
    private void shootGoal(){
        for (int i =0;i < 3; i++){
            turnOnShootingMotor();
            turnOnFlicker(0.6);
            telemetry.addData("flickposition", "0.6");
            telemetry.update();
            sleep(500);
            turnOnFlicker(0.38);
            telemetry.addData("flickposition", "0.38");
            telemetry.update();
            sleep(500);
            turnOffShootingMotor();
            if(i != 2) {
                sleep(7000);
            }
        }

     turnOffShootingMotor();

    }

    void shootGoal(BlueDestination Destination) {
        while (opModeIsActive()) {
            turnOnShootingMotor();
            switch (Destination) {
                case A1:
                    sleep(600);
                    break;
                case B2:
                    sleep(650);
                    break;
                case C1:
                    sleep(700);
                    break;
            }
            //Flicker is on and shoots ring.
            turnOnFlicker(0.6);
            sleep(2000);
            turnOnFlicker(0.38);
            turnOffShootingMotor();
            turnOffFlicker();
            break;
        }

    }

    public void turnOnShootingMotor() {
        if (opModeIsActive()) {
            robot.shooter.setPower(.955); //instructing the amount of power the shooter must use to shoot
            sleep(300); // wait for motor to get good speed.
        }
    }

    public void turnOffShootingMotor() {
        if (opModeIsActive()) {
            robot.shooter.setPower(0);
        }
    } //switching power to 0 to stop movement

    public void turnOnFlicker(Double position) {
        if (opModeIsActive()) {
            robot.flick.setPosition(position); //moves flicker at high speed to a certain location
        }
    }

    public void turnOffFlicker() {} //bringing flicker back to starting position

    */
/*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired
     *  position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     *//*



    public void setShootingPosition() {
    // move to right location for shooting

    }

    //Changing the route to shooting line depending on where the wobble goal was dropped.
    public void gotoShootingLine(BlueDestination Destination) {
        switch (Destination) {
            case A1:
                messageDisplay("case A1");
                //driveReverse(DRIVE_SPEED, 1.75f * TILE);
                DriveforTime(-0.75,.35,5);
                //slideRight(0.8f * TILE);
                //sleep(200);
                //driveReverse(DRIVE_SPEED, 0.95f * TILE);
                //turnLeftTime(0.5, 300);
                break;
            case B2:
                messageDisplay("case B2");
                //driveReverse(DRIVE_SPEED, 1.65f * TILE);
                //sleep(300);
                //slideLeft(1.2f * TILE);
                break;
            case C1:
                messageDisplay("case C1");
                //driveReverse(DRIVE_SPEED, 0.95f * TILE);
                slideRight(0.9f * TILE);
                DriveforTime(1.0,.40,5);
                //sleep(15);
                //turnLeftTime(0.5, 50);
                break;


        }

    }
// Functions that will allow the movement
    public void slideRight(float distance) {

        float tm = (distance / TILE);

        strafeRightTime(1.0f, (tm / 1.5)*1000);
    }

    public void slideLeft(float distance) {
        float tm = (distance / TILE);

        strafeLeftTime(1.0f, (tm / 1.5)*1000);
       //strafeLeft(1.0f, distance, tm / 1.5);


    }

    public void turnRight(float distance) {
        encoderDrive(DRIVE_SPEED, distance, -distance, 30);
    }

    public void turnLeft(float distance) {
        encoderDrive(DRIVE_SPEED, -distance, distance, 30);
    }

    public void dropOffWobbleGoal() {
        messageDisplay("Inside Drop Off goal");
        moveArmDown(-ARM_DROP_OFF_ANGLE);
        clawOpen();
        moveArmUp(ARM_RETRACT_ANGLE);


    }
//Functions that will control the claw
    public void clawOpen() {
        clawControl(0);
    }

    public void clawClose() {
        if (opModeIsActive()) {
            clawControl(1);
        }
    }

    public void clawControl(double position) {
        if (opModeIsActive()) {
            robot.claw.setPosition(position);
            sleep(2000);
        }
    }
//Function that makes the arm move downwards
    public void moveArmDown(double angle) {

        if (opModeIsActive()) {

            robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.arm.setPower(1.0);
            sleep(200);

            robot.arm.setPower(0);
        }
      //  runtime.reset();

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.




        telemetry.update();
    }

//Function that moves arm up
    public void moveArmUp(double angle) {
        messageDisplay("Inside Move arm up"); //Tells us the arm is moving
        if (opModeIsActive()) {
            robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.arm.setPower(-1.0);
            sleep(300);

            robot.arm.setPower(0);
        }
        //runtime.reset();

    } //After the motors have slept for 1000 miliseconds, the arm's finished position will be the most recent position and will update frequently


    public BlueDestination getDestination(String startPosition) {
        int rings = 0;

        switch (startPosition) {
            case "F1":
                turnRight(0.5f * TILE);
                rings = findRingCount();
                turnLeft(0.5f * TILE);
                break;
                //if the starting position is F1 the robot will turn towards the rings, and the amount of rings will stay 0 until detected otherwise
            case "F2":
                //driveForward(0.3f, 0.45 * TILE);
                DriveforTime(1.0, .32, 10 );
               // strafeRight(1.0,6,0.2);
                rings = findRingCount();
                turnOffFlicker();
                break;
            //if the starting position is F2 the robot will turn towards the rings, and the amount of rings will stay 0 until detected otherwise
            case "F3":
                driveForward(DRIVE_SPEED,19.0f);
                slideLeft(TILE);
                rings = findRingCount();
               /// turnRight(0.5, 0.5f * TILE);
                break;
            //if the starting position is F3 the robot will turn towards the rings, and the amount of rings will stay 0 until detected otherwise
        }


        BlueDestination destination = BlueDestination.C1;
        switch (rings) {
            case 0:
                destination = BlueDestination.C1;
                break;
                //if it detected 0 rings it will drive towards C1
            case 1:
                destination = BlueDestination.B2;
                break;
                //if it detected 1 ring it will drive towards B2
            case 4:
                destination = BlueDestination.A1;
                break;
                //if it detected 4 rings it will drive towards A1

        }
        // messageDisplay("Before Exit "+ destination);

        return destination;
        //Tells us the destination according to ring count.
    }

    public void messageDisplay(String message) {
        telemetry.addData("Inside Method", "%s", message);
        telemetry.update();
        //allows a peice of text while a method is running to show on the phone
    }

    public int findRingCount() {
        int rings = 0; //sets default ring amount as 0
        initVuforia();
        initTfod();
        //initializes tensor flow and vuforia
        if (tfod != null) {
            tfod.activate(); //if tensor flow is not equal to null it activates
        }
        //runtime.reset();
        if (opModeIsActive()) {
            while (opModeIsActive() && runtime.seconds() < OBJECT_DETECT_WAIT) {
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {

                        if (updatedRecognitions.size() > 1) {
                            rings = 4;
                            telemetry.addData("Ring Count", rings);
                        }
                        */
/* if tensorflow is not equal to null then it will list the recognition and update it every time it changes,
                        if the size of the updated recognition is greater than 1 it is 4 rings.
                         *//*

                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) { //
                            float height = recognition.getHeight();

                            telemetry.addData("Height",
                                    height);

                            //sleep(3000);

                            if (height >= OBJECT_HEIGHT_LIMIT) {
                                rings = 4;
                                telemetry.addData("Ring Count", rings);
                            }
                            if (height < OBJECT_HEIGHT_LIMIT) {
                                rings = 1;
                                telemetry.addData("Ring Count", rings);

                            }
                            //the recognition's heights are measured and displayed on the phone, if the height is greater or equal to the height
                            //limit then the amount of rings is four, and the amount of rings is displayed. same goes for 1 ring except the height must
                            //less than the object height limit.


                        }
                        telemetry.update();
                        //will update frequently
                    }
                }
            }
        }

        telemetry.addData("Rings detected : ", rings);
        telemetry.update();
        return (rings);
        //provides the amount of rings it detected and updates it when it has changed.
    }

    public String gotoDestination(String startPosition, BlueDestination Destination) {

        switch (Destination) {
            case C1:
                telemetry.addData("destination", "C1");
                telemetry.update();
                goToDestination_C1(startPosition);
                break;
            case B2:
                telemetry.addData("destination", "B2");
                telemetry.update();
                goToDestination_B2(startPosition);
                break;
            case A1:
                telemetry.addData("destination", "A1");
                telemetry.update();
                goToDestination_A1(startPosition);
                break;
        }
        //Makes a function that allows us to easily choose a destination easily from the start position.


        return ("");
    }

//Instructing the robot how to get to each destination
    void goToDestination_C1(String startPosition) {
           slideLeft(0.9f*TILE);
         // DriveforTime(1.0,1.1,5);
           slideRight(.7f * TILE);
          //DriveforTime(1.0,.5,5);
        driveForward(1, .65f * TILE);
//        sleep(500);
        slideLeft(1.0f * TILE);
        //dropOffWobbleGoal();
          slideRight(.5f * TILE);
        //DriveforTime(1.0,.25,5);
    }

    void goToDestination_B2(String startPosition) {
        slideRight(0.70f * TILE);
        driveForward(1, 1.5 * TILE);
        //DriveforTime(1.0,1.1,5);
        sleep(100);
        slideLeft(.5f * TILE);
        //dropOffWobbleGoal();

    }

    void goToDestination_A1(String startPosition) {
        slideRight(0.75f * TILE);
        //DriveforTime(1.0,1.7,5);
        driveForward(1, 2.5 * TILE);
        sleep(100);
        slideLeft(1.75f * TILE);
        //dropOffWobbleGoal();
       //slideLeft(2.0f * TILE);

    }

    public String getStartPosition() {
        return ("");
    }

    //instructs robot how to move in certain ways
    public void driveReverse (double speed, double distance) {
        encoderDriveReverse(DRIVE_SPEED, distance, distance, 30);

    }

    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int newbackLeftTarget;
        int newbackRightTarget;
        int newfrontLeftTarget;
        int newfrontRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = robot.leftDrive.getCurrentPosition() + moveCounts;
            newRightTarget = robot.rightDrive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftDrive.setTargetPosition(newbackLeftTarget);
            robot.rightDrive.setTargetPosition(newbackRightTarget);

            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftDrive.setPower(speed);
            robot.rightDrive.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.leftDrive.setPower(leftSpeed);
                robot.rightDrive.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    }

    public void driveForward(double speed, double distance) {
        encoderDrive(DRIVE_SPEED, distance, distance, 30);
    }

    public void turnRight(double speed, double distance) {
        encoderDrive(DRIVE_SPEED, distance, -distance, 30);
    }

    public void turnLeft(double speed, double distance) {
        encoderDrive(DRIVE_SPEED, -distance, distance, 30);
    }

    public void turnRightTime(double speed, long time) {
        resetStartTime();
        robot.back_left.setPower(speed);
        robot.back_right.setPower(-speed);
        robot.front_left.setPower(speed);
        robot.front_right.setPower(-speed);
        while(opModeIsActive() && runtime.milliseconds() < time)
        {

        }




    stopAllMotors();

}
    public void turnLeftTime(double speed, long time) {
        robot.back_left.setPower(-speed);
        robot.back_right.setPower(speed);
        robot.front_left.setPower(-speed);
        robot.front_right.setPower(speed);

        while(opModeIsActive() && runtime.milliseconds() < time)
        {

        }

        stopAllMotors();

    }

    public void stopAllMotors() { //stops all motors
        robot.back_left.setPower(0);
        robot.back_right.setPower(0);
        robot.front_left.setPower(0);
        robot.front_right.setPower(0);
    }

    public void DriveforTime(double speed,
                             double seconds,
                             double timeoutS) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            runtime.reset();
            robot.back_right.setPower(speed);
            robot.back_left.setPower(speed);
            robot.front_right.setPower(speed);
            robot.front_left.setPower(speed);
            seconds = seconds*1000;
            double i = 0;
            while (opModeIsActive() && i < seconds &&
                    (runtime.seconds() < timeoutS)) {
                sleep(1);
                i++;
            }

            robot.front_right.setPower(0);    // Stop all motion;
            robot.back_right.setPower(0);
            robot.front_left.setPower(0);
            robot.back_left.setPower(0);

        }
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newbackLeftTarget;
        int newbackRightTarget;
        int newfrontLeftTarget;
        int newfrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            robot.front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            robot.back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //    robot.front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         //   robot.front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



            // Determine new target position, and pass to motor controller
            newbackLeftTarget = robot.back_left.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newbackRightTarget = robot.back_right.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newfrontLeftTarget = robot.front_left.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newfrontRightTarget = robot.front_right.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.back_right.setPower(speed*RIGHT_BACK_COEFF);
            robot.back_left.setPower(speed*LEFT_BACK_COEFF);
            robot.front_right.setPower(speed*RIGHT_FRONT_COEFF);
            robot.front_left.setPower(speed*LEFT_FRONT_COEFF);



            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            telemetry.addData("Beforeloop","Running at %7d :%7d : %7d :%7d",
                    robot.back_left.getCurrentPosition(),
                    robot.back_right.getCurrentPosition(),
                    robot.front_left.getCurrentPosition(),
                    robot.front_right.getCurrentPosition());



            while (opModeIsActive() && (robot.back_left.getCurrentPosition() < newbackLeftTarget) && (robot.back_right.getCurrentPosition() < newbackRightTarget) &&
                    (runtime.seconds() < timeoutS))

            {
                //Provides current position and updates it every time it changes.
                telemetry.addData("CurrentPositon", "Running at %7d :%7d ",
                        robot.back_left.getCurrentPosition(),
                        robot.back_right.getCurrentPosition());
                telemetry.update();
            }

            robot.front_right.setPower(0);    // Stop all motion;
            robot.back_right.setPower(0);
            robot.front_left.setPower(0);
            robot.back_left.setPower(0);
            telemetry.addData("Path2", "Running at %7d :%7d  :%7d  :%7d",
                    robot.front_left.getCurrentPosition(),
                    robot.front_right.getCurrentPosition(),
                    robot.back_left.getCurrentPosition(),
                    robot.back_right.getCurrentPosition());
            telemetry.update();




                //  sleep(250);   // optional pause after each move
            }
        }

    public void encoderDriveReverse(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newbackLeftTarget;
        int newbackRightTarget;
        int newfrontLeftTarget;
        int newfrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            robot.front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            // Determine new target position, and pass to motor controller
            newbackLeftTarget = robot.back_left.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newbackRightTarget = robot.back_right.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newfrontLeftTarget = robot.front_left.getCurrentPosition() - (int) (leftInches * COUNTS_PER_INCH);
            newfrontRightTarget = robot.front_right.getCurrentPosition() - (int) (rightInches * COUNTS_PER_INCH);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.back_right.setPower(-speed*RIGHT_BACK_COEFF);
            robot.front_right.setPower(-speed*RIGHT_FRONT_COEFF);
            robot.back_left.setPower(-speed*LEFT_BACK_COEFF);
            robot.front_left.setPower(-speed*LEFT_FRONT_COEFF);



            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            while (opModeIsActive() && (robot.front_left.getCurrentPosition() > newfrontLeftTarget) && (robot.front_right.getCurrentPosition() > newfrontRightTarget) &&
                    (runtime.seconds() < timeoutS))

            {
                //tells location/position of every wheel and updates every change.
                telemetry.addData("CurrentPositon", "Running at %7d :%7d  :%7d  :%7d",
                        robot.front_left.getCurrentPosition(),
                        robot.front_right.getCurrentPosition(),
                        robot.back_left.getCurrentPosition(),
                        robot.back_right.getCurrentPosition());
                telemetry.update();
            }

            robot.front_right.setPower(0);    // Stop all motion;
            robot.back_right.setPower(0);
            robot.front_left.setPower(0);
            robot.back_left.setPower(0);
            //tells location/position of every wheel and updates every change while going to path2
            telemetry.addData("Path2", "Running at %7d :%7d  :%7d  :%7d",
                    robot.front_left.getCurrentPosition(),
                    robot.front_right.getCurrentPosition(),
                    robot.back_left.getCurrentPosition(),
                    robot.back_right.getCurrentPosition());
            telemetry.update();
        }
    }



    public void strafeRightTime(double speed,double timeMilliSecs) {
        int newbackLeftTarget;
        int newbackRightTarget;
        int newfrontLeftTarget;
        int newfrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            robot.front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.back_right.setPower(speed*(RIGHT_BACK_COEFF+ .2F));
            robot.back_left.setPower(-speed*LEFT_BACK_COEFF);
            robot.front_left.setPower(speed*LEFT_FRONT_COEFF);
            robot.front_right.setPower(-speed*(RIGHT_FRONT_COEFF+.2F));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive()  &&(runtime.milliseconds() < timeMilliSecs))

            {

                telemetry.addData("RightFront,LeftFront","Running to %7d :%7d",robot.front_right.getCurrentPosition(),robot.front_left.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.front_right.setPower(0);
            robot.front_left.setPower(0);
            robot.back_right.setPower(0);
            robot.back_left.setPower(0);
            messageDisplay("currentPosition:"+ Integer.toString(robot.back_left.getCurrentPosition()));

        }
    }
    public void strafeRight(double speed,
                           double distanceInches,
                           double timeoutS) {
        int newbackLeftTarget;
        int newbackRightTarget;
        int newfrontLeftTarget;
        int newfrontRightTarget;

        robot.front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        newfrontLeftTarget = robot.front_left.getCurrentPosition() + (int) (distanceInches * COUNTS_PER_INCH);





        robot.front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {




            // reset the timeout time and start motion.
            runtime.reset();
            robot.back_right.setPower(speed*(RIGHT_BACK_COEFF+ .2F));
            robot.back_left.setPower(-speed*LEFT_BACK_COEFF);
            robot.front_left.setPower(speed*LEFT_FRONT_COEFF);
            robot.front_right.setPower(-speed*(RIGHT_FRONT_COEFF+.2F));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (robot.front_left.getCurrentPosition() < newfrontLeftTarget) &&
                    (runtime.seconds() < timeoutS))

            {

                telemetry.addData("RightFront,LeftFront","Running to %7d :%7d",robot.front_right.getCurrentPosition(),robot.front_left.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.front_right.setPower(0);
            robot.front_left.setPower(0);
            robot.back_right.setPower(0);
            robot.back_left.setPower(0);
            messageDisplay("currentPosition:"+ Integer.toString(robot.back_left.getCurrentPosition()));

        }
    }

    public void strafeLeftTime(double speed,

                           double timeMilliSeconds) {
        int newbackLeftTarget;
        int newbackRightTarget;
        int newfrontLeftTarget;
        int newfrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            robot.front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // reset the timeout time and start motion.
            runtime.reset();
        //  robot.back_right.setPower(-speed * (RIGHT_BACK_COEFF + .2F));
            robot.back_right.setPower(-speed * (RIGHT_BACK_COEFF ));
            robot.back_left.setPower(speed * LEFT_BACK_COEFF);
            robot.front_left.setPower(-speed * LEFT_FRONT_COEFF);
            //robot.front_right.setPower(speed * (RIGHT_FRONT_COEFF + .2F));
            robot.front_right.setPower(speed * (RIGHT_FRONT_COEFF ));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.milliseconds() < timeMilliSeconds))

            //&& (robot.front_left.getCurrentPosition() < newfrontLeftTarget) && (robot.front_right.getCurrentPosition() < newfrontRightTarget) )


            {

                telemetry.addData("RightFront,LeftFront", "Running to %7d :%7d", robot.front_right.getCurrentPosition(), robot.front_left.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.front_right.setPower(0);
            robot.front_left.setPower(0);
            robot.back_right.setPower(0);
            robot.back_left.setPower(0);
            messageDisplay("currentPosition:" + Integer.toString(robot.back_left.getCurrentPosition()));

        }
    }
    public void strafeLeft(double speed,
                             double distanceInches,
                             double timeoutS) {
        int newbackLeftTarget;
        int newbackRightTarget;
        int newfrontLeftTarget;
        int newfrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            robot.front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            newfrontRightTarget = robot.front_right.getCurrentPosition() + (int) (distanceInches * COUNTS_PER_INCH);


            robot.back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER  );
            robot.front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            // reset the timeout time and start motion.
            runtime.reset();

            robot.back_right.setPower(-speed*(RIGHT_BACK_COEFF +.2F));
            robot.back_left.setPower(speed*LEFT_BACK_COEFF);
            robot.front_left.setPower(-speed*LEFT_FRONT_COEFF);
            robot.front_right.setPower(speed*(RIGHT_FRONT_COEFF+.2F));
            while (opModeIsActive() && (robot.front_right.getCurrentPosition() < newfrontRightTarget) &&
                    (runtime.seconds() < timeoutS))

            {
                //Provides current position and updates it every time it changes.
                telemetry.addData("CurrentPositon", "Running at %7d :%7d  :%7d  :%7d",
                        robot.front_left.getCurrentPosition(),
                        robot.front_right.getCurrentPosition(),
                        robot.back_left.getCurrentPosition(),
                        robot.back_right.getCurrentPosition());
                telemetry.update();
            }

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.


            // Stop all motion;
            robot.front_right.setPower(0);
            robot.front_left.setPower(0);
            robot.back_right.setPower(0);
            robot.back_left.setPower(0);
            messageDisplay("currentPosition:"+ Integer.toString(robot.back_left.getCurrentPosition()));

        }
    }

  private void initVuforia() {
  VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

    parameters.vuforiaLicenseKey = VUFORIA_KEY;
    parameters.cameraName = robot.webcam;
            //robot.hwMap.get(WebcamName .class, "Webcam");

    //  Instantiate the Vuforia engine
    vuforia = ClassFactory.getInstance().createVuforia(parameters);

    // Loading trackables is not necessary for the TensorFlow Object Detection engine.
}

    */
/**
     * Initialize the TensorFlow Object Detection engine.
     *//*

   private void initTfod() {
        appIdentifier = robot.hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", robot.hwMap.appContext.getPackageName());


        int tfodMonitorViewId =appIdentifier;
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }





    }




*/
