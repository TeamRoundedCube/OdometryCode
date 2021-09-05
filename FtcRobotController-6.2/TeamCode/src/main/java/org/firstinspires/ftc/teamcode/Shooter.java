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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="MaxShooter", group="Linear Opmode")

public class Shooter extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotorEx shooterMotor;
    private Double maxElapsedTime=0.0  ;
    double maxvelocity= 0.0;
    private Servo flick;
    private HardwareFullBot robot;
    private HardwareMap hwmap=null;
    @Override
    public void runOpMode() {
start();
    }
    public Shooter(HardwareFullBot p_robot){
        robot = p_robot;
      //  telemetry.addData("From Shooter :","Contructer called");
      //  telemetry.update();
      //  sleep(1000);
    }
    public Shooter(){}
    // - Motor Speed set to 1560 and shooting speed 1540
    // - Shooting Speed to 1550
    public void shoot(Integer shootingSpeed ,Integer motorticksPerSec,Integer shootCount) {


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
      //  telemetry.addLine("TEST");
      //  telemetry.update();
        //sleep(1000);
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


            while (opModeIsActive() && runtime.seconds() < 10.0 && i < shootCount) {

                // Setup a variable for each drive wheel to save power level for telemetry
                double currentvelocity = shooterMotor.getVelocity();
                telemetry.addData("Current Velocity at time ", "Velocity(%.2f), time (%.2f)", currentvelocity, runtime.milliseconds());
                telemetry.update();

                if ( currentvelocity== shootingSpeed) {

                    flick.setPosition(.60);
                    sleep(500);
                    flick.setPosition(0.38);
                    sleep(300);


                    // shooterMotor.setPower(0);

                    i++;
                }
                // Show the elapsed game time and wheel power.


            }
            shooterMotor.setPower(0);
            telemetry.addData("MaxVelocity at time ", "Velocity(%.2f), time (%.2f)", maxvelocity, maxElapsedTime);
            telemetry.update();
            //sleep(10000);
        }
    }
}

