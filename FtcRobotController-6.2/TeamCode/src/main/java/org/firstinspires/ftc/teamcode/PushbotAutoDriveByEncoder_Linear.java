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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Disabled
@Autonomous(name="Pushbot:AutotoLine", group="Pushbot")

public class PushbotAutoDriveByEncoder_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 537;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.9;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = .80;


    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Start",  "Starting at %7d :%7d :%7d :%7d",
                          robot.front_left.getCurrentPosition(),
                          robot.front_right.getCurrentPosition(),
                robot.back_left.getCurrentPosition(),
                robot.back_right.getCurrentPosition()
        );


        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
       encoderDrive(DRIVE_SPEED,  60, 60, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
       // slide(1.0,10,10,30);
       //slide(DRIVE_SPEED,      60,60, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
       // encoderDrive(DRIVE_SPEED, -83, -83, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
        // encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout


        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newbackLeftTarget;
        int newbackRightTarget;
        int newfrontLeftTarget;
        int newfrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newbackLeftTarget = robot.back_left.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newbackRightTarget = robot.back_right.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newfrontLeftTarget = robot.front_left.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newfrontRightTarget = robot.front_right.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);

            robot.front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.back_left.setTargetPosition(newbackLeftTarget);
           robot.back_right.setTargetPosition(newbackRightTarget);
           robot.front_right.setTargetPosition(newfrontRightTarget);
            robot.front_left.setTargetPosition(newfrontLeftTarget);

            // Turn On RUN_TO_POSITION
           robot.front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.

            runtime.reset();
           robot.back_right.setPower(Math.abs(speed));
           robot.back_left.setPower(Math.abs(speed));
          robot.front_left.setPower(Math.abs(speed));
          robot.front_right.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS)
                &&
                (  robot.front_right.isBusy())
        )
            {
                //  brake(robot,0.7);
                // Display it for the driver.

               // if( Math.abs((robot.front_left.getCurrentPosition()/ robot.front_left.getTargetPosition())) > 0.7)
               // {
                //if (robot.front_left.getCurrentPosition() > (int)(newfrontLeftTarg6Eet*0.7)){
                    //speed = 1.0;

                /*    robot.back_left.setPower(Math.abs(speed));
                    robot.front_left.setPower(Math.abs(speed));
                    robot.front_right.setPower(Math.abs(speed));
                    robot.back_right.setPower(Math.abs(speed));*/
                //}


              //  }

                telemetry.addData("target", "Running to %7d :%7d   :%7d  :%7d", newbackLeftTarget
                        , newbackRightTarget
                        , newfrontLeftTarget
                        , newfrontRightTarget);
                telemetry.addData("CurrentPositon", "Running at %7d :%7d  :%7d  :%7d",
                        robot.front_left.getCurrentPosition(),
                        robot.front_right.getCurrentPosition(),
                        robot.back_left.getCurrentPosition(),
                        robot.back_right.getCurrentPosition());
                telemetry.update();

            }

                // Stop all motion;
                robot.front_left.setPower(0);
                robot.back_right.setPower(0);
                robot.front_right.setPower(0);

                robot.back_left.setPower(0);

            telemetry.addData("Path2", "Running at %7d :%7d  :%7d  :%7d",
                    robot.front_left.getCurrentPosition(),
                    robot.front_right.getCurrentPosition(),
                    robot.back_left.getCurrentPosition(),
                    robot.back_right.getCurrentPosition());
            telemetry.update();
sleep(120000);

                // Turn off RUN_TO_POSITION


                //  sleep(250);   // optional pause after each move
            }
        }

    public void slide(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newbackLeftTarget;
        int newbackRightTarget;
        int newfrontLeftTarget;
        int newfrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            robot.front_left.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            robot.front_right.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
            robot.back_left.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
            robot.back_right .setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

            newbackLeftTarget = robot.back_left.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newbackRightTarget = robot.back_right.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newfrontLeftTarget = robot.front_left.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newfrontRightTarget = robot.front_right.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.back_left.setTargetPosition(newbackLeftTarget);
            robot.back_right.setTargetPosition(newbackRightTarget);
            robot.front_right.setTargetPosition(newfrontRightTarget);
             //  robot.front_left.setTargetPosition(newfrontLeftTarget);

            // Turn On RUN_TO_POSITION
         //   robot.front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            robot.back_right.setPower(Math.abs(speed));
            robot.back_left.setPower(Math.abs(speed));
           // robot.front_left.setPower(Math.abs(speed));
            robot.front_right.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.front_right.isBusy() && robot.front_left.isBusy() && robot.back_right.isBusy() && robot.back_left.isBusy()  ) ) {

                // Display it for the driver.
                //brake(robot,0.7);
                telemetry.addData("Target",  "Running to %7d :%7d   :%7d  :%7d", newbackLeftTarget
                        , newbackRightTarget
                        ,newfrontLeftTarget
                        ,newfrontRightTarget);
                telemetry.addData("CurrentPosition",  "Running at %7d :%7d  :%7d  :%7d",
                        robot.front_left.getCurrentPosition(),
                        robot.front_right.getCurrentPosition(),
                        robot.back_left.getCurrentPosition(),
                        robot.back_right.getCurrentPosition());
                telemetry.update();

            }
            sleep(2000);
            // Stop all motion;
            robot.front_right.setPower(0);
            robot.front_left.setPower(0);
            robot.back_right.setPower(0);
            robot.back_left.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

public void brake(HardwarePushbot robot, double percentBrakingDistance)
{

    int currentTick =robot.back_left.getCurrentPosition();
    int targetTick = robot.back_left.getTargetPosition();
    double currentBrakingPercentDistance = currentTick/targetTick;
    if ((currentBrakingPercentDistance) > percentBrakingDistance){

        double adjustedPower = Math.abs(1- 1/((1-currentBrakingPercentDistance)*100));
        robot.back_left.setPower(0.5);
        robot.back_right.setPower(0.5);
        robot.front_right.setPower(0.5);
        robot.front_left.setPower(0.5);


    }

    }
}



