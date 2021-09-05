package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.lang.*;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;

//@TeleOp(name="VikrantRajivTeleOp", group="FullBot")

public class VikrantRajivTeleOp extends OpMode {

    //HardwareMap hwMap;

    //Motors
    DcMotor backLeftWheel;
    DcMotor backRightWheel;
    DcMotor frontLeftWheel;
    DcMotor frontRightWheel;
    DcMotor arm;

    //Servos
    Servo claw;
    Servo flick;
    Servo drop;

    double backLeftWheelPower;
    double backRightWheelPower;
    double frontLeftWheelPower;
    double frontRightWheelPower;

    double backLeftWheelTurningPower;
    double backRightWheelTurningPower;
    double frontLeftWheelTurningPower;
    double frontRightWheelTurningPower;

    double backLeftWheelStrafePower;
    double backRightWheelStrafePower;
    double frontLeftWheelStrafePower;
    double frontRightWheelStrafePower;

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    @Override
    public void init() {

        //robot.init(hardwareMap);



        //Motors
        backLeftWheel = hardwareMap.dcMotor.get("back_left");
        backRightWheel = hardwareMap.dcMotor.get("back_right");
        frontLeftWheel = hardwareMap.dcMotor.get("front_left");
        frontRightWheel = hardwareMap.dcMotor.get("front_right");

        backRightWheel.setDirection(DcMotor.Direction.REVERSE);
        frontLeftWheel.setDirection(DcMotor.Direction.REVERSE);

        //Servos
        flick = hardwareMap.get(Servo.class, "flick" );
        drop = hardwareMap.get(Servo.class, "drop");
        claw = hardwareMap.get(Servo.class, "claw");
        arm = hardwareMap.get(DcMotor.class, "arm");

    }

    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {

        //Setting controls on gamepad1
        backLeftWheelPower = -gamepad1.left_stick_y;
        backRightWheelPower = -gamepad1.left_stick_y;
        frontLeftWheelPower = -gamepad1.left_stick_y;
        frontRightWheelPower = -gamepad1.left_stick_y;

        backLeftWheelTurningPower = gamepad1.right_stick_x;
        backRightWheelTurningPower = gamepad1.right_stick_x;
        frontLeftWheelTurningPower = gamepad1.right_stick_x;
        frontRightWheelTurningPower = gamepad1.right_stick_x;

        backLeftWheelStrafePower = gamepad1.left_stick_x;
        backRightWheelStrafePower = gamepad1.left_stick_x;
        frontLeftWheelStrafePower = gamepad1.left_stick_x;
        frontRightWheelStrafePower = gamepad1.left_stick_x;


        telemetry.addData("leftx:", java.lang.Math.abs(gamepad1.left_stick_x));
        telemetry.addData("lefty:", java.lang.Math.abs(gamepad1.left_stick_y));
        telemetry.update();

        // Diagonal
        boolean diagonalY = (java.lang.Math.abs(gamepad1.left_stick_y) > 0.3 && java.lang.Math.abs(gamepad1.left_stick_y) < 0.8);
        boolean diagonalX = (java.lang.Math.abs(gamepad1.left_stick_x) > 0.3 && java.lang.Math.abs(gamepad1.left_stick_x) < 0.8);
        boolean diagonal = diagonalY && diagonalX;

        //Forward and backward - Left Stick
        backLeftWheel.setPower(-backLeftWheelPower);
        backRightWheel.setPower(-backRightWheelPower);
        frontLeftWheel.setPower(frontLeftWheelPower);
        frontRightWheel.setPower(frontRightWheelPower);

        //Strafe - Left Stick
        backLeftWheel.setPower(-backLeftWheelStrafePower);
        backRightWheel.setPower(backRightWheelStrafePower);
        frontLeftWheel.setPower(-frontLeftWheelStrafePower);
        frontRightWheel.setPower(frontRightWheelStrafePower);

        //Turning - Right Stick
        backLeftWheel.setPower(backLeftWheelTurningPower);
        backRightWheel.setPower(-backRightWheelTurningPower);
        frontLeftWheel.setPower(-frontLeftWheelTurningPower);
        frontRightWheel.setPower(frontRightWheelTurningPower);

        //Servos
        //Shooting
        if (gamepad2.right_bumper) {
            flick.setPosition(0.38);
            flick.setPosition(0.00);
        }
        if (gamepad2.right_trigger > 0.2) {
            flick.setPosition(0.6);
            flick.setPosition(0.00);
        }

        //Arm
        int downPosition = 1464;
        int upPosition = 0;
        int wallPosition = 675;


        if (gamepad1.dpad_down) {
            arm.setPower(-0.8);
            arm.setTargetPosition(downPosition);
            arm.setMode(RUN_TO_POSITION);
            sleep(1000);
            claw.setPosition(0.0);
        } else if (gamepad1.dpad_up) {
            claw.setPosition(1.0);
            sleep(1000);
            //robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setPower(0.8);
            arm.setTargetPosition(upPosition);
            arm.setMode(RUN_TO_POSITION);
        } else if (gamepad1.dpad_left || gamepad1.dpad_right) {
            arm.setPower(-0.25);
            arm.setTargetPosition(wallPosition);
            arm.setMode(RUN_TO_POSITION);
            sleep(1000);
            claw.setPosition(0);
        }









    }
}