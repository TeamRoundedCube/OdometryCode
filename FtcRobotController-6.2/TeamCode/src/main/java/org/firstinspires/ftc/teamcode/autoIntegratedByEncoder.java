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



/*
      G O A L
 |A1|A2}A3|A4|A5|A6|
 |B1|B2}B3|B4|B5|B6|
 |C1|C2}C3|C4|C5|C6|
 |D1|D2}D3|D4|D5|D6|
 |E1|E2}E3|E4|E5|E6|
 |F1|F2}F3|F4|F5|F6|
     S T A R T

 */
// testing new branch
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

@Autonomous(name="AutonomousDecember", group="FullBot")

public class autoIntegratedByEncoder extends LinearOpMode {

    // Declare OpMode members.
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
    private static final double ARM_DROP_OFF_ANGLE = 8.0f;
    private static final double ARM_RETRACT_ANGLE = 92.0f;
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "1Ring";
    private static final String LABEL_SECOND_ELEMENT = "4Ring";
    private static final String VUFORIA_KEY =
            "AUtexuD/////AAABmQIASsSYI0K7oKlLQXSWAklWJXXAgCte5uORbUIju1rDuc89XOps0SeZL95/obR5ocozZUTPbARiP/Jr6tkoyPVWNpQFZPCgwYoSmBv/wwAgOq21dkfd/BF8iR4diFLwOyyc9OY//6yF4nSM1qttUf+g576o/IHb+DLzMAzcmYwINNhBqIWxpuGDs7mDT0hOAllHo3kFRAeUf3pRl/1uzVOkUfD16IGhPy/ZSqHIiAvEgtZiGU+JC3h32hLDclTJka3KTb4Fo6fyUMpotamPiDA19zT+jdcNg8LJSDtQlymeDXVPGJs5rRwrQ/RPfKacFare/F+jrC7GWiS/jxDok4mJ5HIoRwMnKUeJTKl5ApCW";
    private static final int OBJECT_DETECT_WAIT = 1000;
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
    static final double COUNTS_PER_INCH = 87;
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

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        // Send telemetry message to ignify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        robot.claw.setPosition(1);
        sleep(2000);
        robot.flick.setPosition(0.38);
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
           // telemetry.addData("destination", destination);
            telemetry.addData("running", "running");
            telemetry.update();

           gotoDestination(startPosition, destination); // Drive to Wobble Goal Delivery area           return A , B,C drive to square
          dropOffWobbleGoal(); // Lower Arm open claw
          //comeBackToPickWobble(destination);

            gotoShootingLine(destination); // Go to shooting spot

           //driveForward(DRIVE_SPEED, 1f * TILE);
        }

       // sleep(1000);     // pause for servos to moveb

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
    //private void shootGoal(){
       // Shooter sh = new Shooter(robot);
       // sh.start();
       //sh.shoot(1540,1545,3);

     //turnOffShootingMotor();

   // }

    void shootGoal() {
        shoot(1540,1540,3);
        //Savita changed 1540 to 1580 then 1570 then 1580 then Kyran changed to 1560
/*        while (opModeIsActive()) {
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
        }*/

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

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired
     *  position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */


    public void setShootingPosition() {
    // move to right location for shooting

    }

    //Changing the route to shooting line depending on where the wobble goal was dropped.
    public void gotoShootingLine(BlueDestination Destination) {
        switch (Destination) {
            case A1:
                messageDisplay("case A1");
                //driveReverse(DRIVE_SPEED, 1.75f * TILE);
                driveReverse(1,20);
                //slideRight(0.8f * TILE);
                //sleep(200);

                //driveReverse(DRIVE_SPEED, 0.95f * TILE);
                //turnLeftTime(0.5, 300);
                break;
            case B2:
                messageDisplay("case B2");
               // driveReverse(DRIVE_SPEED, 16);
                sleep(200);
                slideRight(0.5f * TILE);
                sleep(100);
                driveReverse(.9, 19+42+4);
                sleep(100);
                //slideLeft(1.7f*TILE);
                slideLeft(40.5f); //35  40
                sleep(100);
                moveArmDown(ARM_DROP_OFF_ANGLE);
                turnRightTime(.6,200);
                sleep(100);
                driveForward(.80,19+42+8);
                sleep(100);
                //turnRightTime(1,4000);
                slideRight(.6,22); //12   17
                driveForward(.80,8);  //added  8
                driveReverse(.9, 8);  //added
                moveArmUp(ARM_RETRACT_ANGLE);
                sleep(200);
                //driveForward(.80,8);
                //slideRight(.8f,1.0f*TILE);
                //driveReverse(1.0, 7);
                //moveArmUp(ARM_RETRACT_ANGLE);
                break;
            case C1:
                messageDisplay("case C1");
                //driveReverse(DRIVE_SPEED, 0.95f * TILE);

                //slideRight(0.9f * TILE);
                //driveForward(1, 10);
                sleep(200);
                slideRight(1.3f*TILE);
                sleep(100);
                driveReverse(1.0,42.0);
                sleep(100);
              //  moveArmDown(ARM_DROP_OFF_ANGLE);
             // sleep(100);
                //slideLeft(1.0f*TILE);
                moveArmUp(ARM_RETRACT_ANGLE);
                sleep(100);
                slideLeft(21.0f);
                sleep(100);
                //sleep(4000);
                //moveArmUp(ARM_RETRACT_ANGLE);
                //  driveReverse(1.0,8);
                //sleep(100);
               // moveArmUp(ARM_RETRACT_ANGLE);
                //sleep(300);
                turnLeftTime(.8,200);

               // slideLeft(6.0f);
                sleep(100);
                moveArmDown(ARM_DROP_OFF_ANGLE);
                sleep (200);
                driveForward(.80,19+6+10+14+4);
                sleep(100);
                moveArmUp(ARM_RETRACT_ANGLE);
                driveReverse(.8,6);
                turnRight(.6,6);
                slideRight(1.0f * TILE);
                driveForward(1.0,8);
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

    public void slideRight(double speed , float distance) {

        float tm = (distance / TILE);

        strafeRightTime(speed, (tm / 1.5)*1000);
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
        moveArmDown(ARM_DROP_OFF_ANGLE);
        sleep(100);
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
        robot.arm.setMode(STOP_AND_RESET_ENCODER);
        int newArmDistance = robot.arm.getCurrentPosition() + (int) (angle * COUNTS_PER_INCH);


        if (opModeIsActive()) {

            robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            robot.arm.setPower(.8);
            while(opModeIsActive() &&   robot.arm.getCurrentPosition() < newArmDistance){
                idle();
            }
           // sleep(300);

           robot.arm.setPower(0);
           // sleep(100);
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
                driveForward(0.5,19.0);
                //DriveforTime(1.0, .32, 10 );
               // strafeRight(1.0,6,0.2);
                rings = findRingCount();
               // turnOffFlicker();
                break;
            //if the starting position is F2 the robot will turn towards the rings, and the amount of rings will stay 0 until detected otherwise
            case "F3":
                driveForward(0.5,19.0f);
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
        runtime.reset();
        if (opModeIsActive()) {
            while (opModeIsActive() && runtime.milliseconds() < OBJECT_DETECT_WAIT) {
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {

                        if (updatedRecognitions.size() > 1) {
                            rings = 4;
                            telemetry.addData("Ring Count", rings);
                        }
                        /* if tensorflow is not equal to null then it will list the recognition and update it every time it changes,
                        if the size of the updated recognition is greater than 1 it is 4 rings.
                         */
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) { //
                            float height = recognition.getHeight();

                            telemetry.addData("Height",height);


                                    telemetry.update();

                      //      sleep(3000);

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
       // sleep(3000);
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
        driveForward(0.5,6.0);
        slideLeft(1.15f * TILE);
        driveForward(1,11);

    }

    void goToDestination_B2(String startPosition) {
        slideRight(0.70f * TILE);
        driveForward(1, 45.0);

        slideLeft(.5f * TILE);



    }

    void goToDestination_A1(String startPosition) {
        slideRight(0.75f * TILE);
        //DriveforTime(1.0,1.7,5);
        driveForward(1.0, 72);
        //sleep(100);
        slideLeft(1.9f * TILE);
        //dropOffWobbleGoal();

       //slideLeft(2.0f * TILE);

    }

    public String getStartPosition() {
        return ("");
    }

    //instructs robot how to move in certain ways
    public void driveReverse (double speed, double distance) {
        encoderDriveReverse(speed, distance, distance, 30);

    }

    public void driveForward(double speed, double distance) {
        encoderDrive(speed, distance, distance, 30);
    }

    public void turnRight(double speed, double distance) {
        encoderDrive(speed, distance, -distance, 30);
    }

    public void turnLeft(double speed, double distance) {
        encoderDrive(speed, -distance, distance, 30);
    }

    public void turnRightTime(double speed, long time) {
        resetStartTime();
        robot.back_left.setPower(speed);
        robot.back_right.setPower(-speed);
        robot.front_left.setPower(speed);
        robot.front_right.setPower(-speed);
        while(opModeIsActive() && runtime.milliseconds() < time)
        {
            sleep(1);
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

            robot.back_left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            robot.back_right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            robot.front_left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            robot.front_right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            robot.back_left.setMode(RUN_WITHOUT_ENCODER);
            robot.back_right.setMode(RUN_WITHOUT_ENCODER);
            robot.front_left.setMode(RUN_WITHOUT_ENCODER);
            robot.front_right.setMode(RUN_WITHOUT_ENCODER);

        //    robot.front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         //   robot.front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);




            // Determine new target position, and pass to motor controller
            newbackLeftTarget = robot.back_left.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newbackRightTarget = robot.back_right.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newfrontLeftTarget = robot.front_left.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newfrontRightTarget = robot.front_right.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.back_left.setTargetPosition(newbackLeftTarget);
            robot.back_right.setTargetPosition(newbackRightTarget);
            robot.front_left.setTargetPosition(newfrontLeftTarget);
            robot.front_right.setTargetPosition(newfrontRightTarget);
            /*PIDFCoefficients pbl,pbr,pfl,pfr;

            pbl=robot.back_left.getPIDFCoefficients(RUN_USING_ENCODER);
            pbr= robot.back_right.getPIDFCoefficients(RUN_USING_ENCODER);
            pfl=robot.front_left.getPIDFCoefficients(RUN_USING_ENCODER);
            pfr=robot.front_right.getPIDFCoefficients(RUN_USING_ENCODER);
            //10,0.05,0,0 PIDF original value
            telemetry.addData("Current pbl ", "Proportion (%.2f), Integral (%.2f),Derivate(%.2f), force (%.2f)", pbl.p ,pbl.i,pbl.d,pbl.f);
            telemetry.addData("Current pbr ", "Proportion (%.2f), Integral (%.2f),Derivate(%.2f), force (%.2f)", pbr.p ,pbr.i,pbr.d,pbr.f);
            telemetry.addData("Current pfl ", "Proportion (%.2f), Integral (%.2f),Derivate(%.2f), force (%.2f)", pfl.p ,pfl.i,pfl.d,pfl.f);
            telemetry.addData("Current pfr ", "Proportion (%.2f), Integral (%.2f),Derivate(%.2f), force (%.2f)", pfr.p ,pfr.i,pfr.d,pfr.f);

            telemetry.update();
            sleep(3000);
*/
          /* double NEW_P = 30;
            double NEW_I = 1.0;
            double NEW_D= 0.0;
            double NEW_F = 0.0;
            PIDFCoefficients pidnew = new PIDFCoefficients(NEW_P,NEW_I,NEW_D,NEW_F);
            robot.back_left.setPIDFCoefficients(RUN_USING_ENCODER,pidnew);
             robot.back_right.setPIDFCoefficients(RUN_USING_ENCODER,pidnew);
            robot.front_left.setPIDFCoefficients(RUN_USING_ENCODER,pidnew);
            robot.front_right.setPIDFCoefficients(RUN_USING_ENCODER,pidnew);
*/
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

         /*                   while (opModeIsActive()
                                    && robot.back_left.isBusy()
                                    && robot.back_right.isBusy()
                                    && robot.front_right.isBusy()
                                    && robot.front_left.isBusy()
                                    &&  runtime.seconds() < timeoutS)*/
                                    {
                //Provides current position and updates it every time it changes
                                        telemetry.addData("Curr Velocity at time ", "backleft(%.2f), " +
                                                "backright (%.2f)",
                                               robot.back_left.getVelocity(),
                                              robot.back_right.getVelocity());
                                        //sleep(250);
                telemetry.update();
                idle();
            }

            robot.front_right.setPower(0);    // Stop all motion;
            robot.back_right.setPower(0);
            robot.front_left.setPower(0);
            robot.back_left.setPower(0);
            telemetry.addData("Path2", "Running at %.2f, %7d :%7d  :%7d  :%7d",
                    leftInches,
                    robot.front_left.getCurrentPosition(),
                    robot.front_right.getCurrentPosition(),
                    robot.back_left.getCurrentPosition(),
                    robot.back_right.getCurrentPosition());
            telemetry.update();




           // sleep(3050);   // optional pause after each move
            }
        }

    public void encoderDriveReverse(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newbackLeftTarget;
        int newbackRightTarget;
        int newfrontLeftTarget;
        int newfrontRightTarget;

        robot.back_left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.back_right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.front_left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.front_right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        robot.back_left.setMode(RUN_WITHOUT_ENCODER);
        robot.back_right.setMode(RUN_WITHOUT_ENCODER);
        robot.front_left.setMode(RUN_WITHOUT_ENCODER);
        robot.front_right.setMode(RUN_WITHOUT_ENCODER);

        //    robot.front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //   robot.front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);




        // Determine new target position, and pass to motor controller
        newbackLeftTarget = robot.back_left.getCurrentPosition() - (int) (leftInches * COUNTS_PER_INCH);
        newbackRightTarget = robot.back_right.getCurrentPosition() - (int) (rightInches * COUNTS_PER_INCH);
        newfrontLeftTarget = robot.front_left.getCurrentPosition() - (int) (leftInches * COUNTS_PER_INCH);
        newfrontRightTarget = robot.front_right.getCurrentPosition() - (int) (rightInches * COUNTS_PER_INCH);
        robot.back_left.setTargetPosition(newbackLeftTarget);
        robot.back_right.setTargetPosition(newbackRightTarget);
        robot.front_left.setTargetPosition(newfrontLeftTarget);
        robot.front_right.setTargetPosition(newfrontRightTarget);
        // Ensure that the opmode is still active
        /*if (opModeIsActive()) {
            robot.front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            // Determine new target position, and pass to motor controller
            newbackLeftTarget = robot.back_left.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newbackRightTarget = robot.back_right.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newfrontLeftTarget = robot.front_left.getCurrentPosition() - (int) (leftInches * COUNTS_PER_INCH);
            newfrontRightTarget = robot.front_right.getCurrentPosition() - (int) (rightInches * COUNTS_PER_INCH);*/

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

            while (opModeIsActive()
                    && (robot.front_left.getCurrentPosition() > newfrontLeftTarget)
                    && (robot.front_right.getCurrentPosition() > newfrontRightTarget)
                    && (robot.back_right.getCurrentPosition() > newbackRightTarget)
                    && (robot.back_left.getCurrentPosition() > newbackLeftTarget)
                    && (runtime.seconds() < timeoutS))

            {
                //tells location/position of everyx wheel and updates every change.
                telemetry.addData("CurrentPositon Inside Loop", "Running at %7d :%7d  :%7d  :%7d",
                        robot.front_left.getCurrentPosition(),
                        robot.front_right.getCurrentPosition(),
                        robot.back_left.getCurrentPosition(),
                        robot.back_right.getCurrentPosition());
                telemetry.update();


                idle();
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
            //messageDisplay("currentPosition:"+ Integer.toString(robot.back_left.getCurrentPosition()));

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

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
   private void initTfod() {
        appIdentifier = robot.hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", robot.hwMap.appContext.getPackageName());


        int tfodMonitorViewId =appIdentifier;
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public void shoot(Integer shootingSpeed ,Integer motorticksPerSec,Integer shootCount) {


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        //  telemetry.addLine("TEST");
        //  telemetry.update();
        //sleep(1000);
        DcMotorEx shooterMotor;
        Servo flick;

        shooterMotor = robot.shooterEx;
        flick = robot.flick;

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery


        // Wait for the game to start (driver presses PLAY)
        runtime.reset();

        if (opModeIsActive()) {
            // run until the end of the match (driver presses STOP)

            shooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooterMotor.setMotorEnable();

            shooterMotor.setDirection(DcMotorEx.Direction.FORWARD);
            shooterMotor.setPower(1.0);


            Integer i = 0;
            shooterMotor.setVelocity(motorticksPerSec);
            PIDFCoefficients p;

            p = shooterMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);
         //   telemetry.addData("Current PID ", "Proportion (%.2f), Integral (%.2f),Derivate(%.2f), force (%.2f)", p.p ,p.i,p.d,p.f);
         //   telemetry.update();
         //   sleep(3000);
            while (opModeIsActive() && runtime.seconds() < 10.0 && i < shootCount) {

                // Setup a variable for each drive wheel to save power level for telemetry
                double currentvelocity = shooterMotor.getVelocity();
                telemetry.addData("Current Velocity at time ", "Velocity(%.2f), time (%.2f)", currentvelocity, runtime.milliseconds());
                telemetry.update();
            //    sleep(10);
                if (i==0){ sleep(700);}
                if (currentvelocity >= shootingSpeed-5 && currentvelocity <= shootingSpeed) {

                    flick.setPosition(.61);
                    sleep(500);
                    flick.setPosition(0.38);
                    sleep(300);


                    // shooterMotor.setPower(0);

                    i++;
                }
                // Show the elapsed game time and wheel power.


            }
            shooterMotor.setPower(0);


            //sleep(10000);
        }
    }



    }




