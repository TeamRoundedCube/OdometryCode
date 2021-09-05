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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.hardware.ColorSensor;


/**
 * This is NOT an opmode.
 *
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareFullBot
{
    /* Public OpMode members. */
    public DcMotorEx front_left  =null;
    public DcMotorEx front_right = null;
    public DcMotorEx back_left =null;
    public DcMotorEx back_right =null;
    public DcMotorEx shooterEx;
    public DcMotor shooter;
    public DcMotor arm;
    public DcMotor intake;
    public WebcamName webcam;
    public VoltageSensor vsense;

    public ColorSensor frontColor;
    public ColorSensor color_right;
    public ColorSensor color_left;
    public DistanceSensor distance_left;
    //  public ColorSensor bottomColor;



    public Servo claw;
    public Servo flick;
    public Servo drop;


    /* local OpMode members. */
      HardwareMap hwMap           = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareFullBot(){


    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap=ahwMap;
        webcam = hwMap.get(WebcamName.class, "Webcam");


        // Define and Initialize Motors
 /*
   <Motor name="front_left" port="0"/>

<Motor name="front_right" port="1"/>

<Motor name="back_left" port="2"/>

<Motor name="back_right" port="3"/>
 */

        front_left  = hwMap.get(DcMotorEx.class, "front_left");
        front_right = hwMap.get(DcMotorEx.class, "front_right");
        back_left    = hwMap.get(DcMotorEx.class, "back_left");
        back_right    = hwMap.get(DcMotorEx.class, "back_right");

        front_left.setDirection(DcMotorEx.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        front_right.setDirection(DcMotorEx.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        back_left.setDirection(DcMotorEx.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        back_right .setDirection(DcMotorEx.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        // Set all motors to zero power
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
        PIDFCoefficients p;





        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        front_left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        //arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //frontColor = hwMap.get(ColorSensor.class,"frontcolor");
   //     bottomColor = hwMap.get(ColorSensor.class,"bottomcolor");
        intake = hwMap.get(DcMotor.class, "intake");
        arm = hwMap.get(DcMotor.class, "arm");
        shooter = hwMap.get(DcMotor.class, "shooter");
        shooterEx = hwMap.get(DcMotorEx.class, "shooter");

        // Define and initialize ALL installed servos.
        flick = hwMap.get(Servo.class, "flick");
        drop = hwMap.get(Servo.class, "drop");
        claw = hwMap.get(Servo.class, "claw");

        color_left = hwMap.get(ColorSensor.class, "color_left");
        color_right = hwMap.get(ColorSensor.class, "color_right");
        distance_left = hwMap.get(DistanceSensor.class, "distance_left");



   //frontColor.green();

    }
 }

