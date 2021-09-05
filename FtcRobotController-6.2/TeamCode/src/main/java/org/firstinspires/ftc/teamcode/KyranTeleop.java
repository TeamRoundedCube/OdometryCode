//  ____          ____  ____           ____   ______________                     ______            ____        ____
// |    |       /    /  \   \        /    /  |     _______   \                 /        \         |     \     |   |
// |    |     /    /     \   \     /    /    |   |        |   \              /    / \    \        |      \    |   |
// |    |   /    /        \   \  /    /      |   |_______|    /            /    /    \    \       |       \   |   |
// |    | /    /           \   \/   /        |     ___      _/           /    /       \    \      |    |\  \  |   |
// |    | \    \            |     |          |    |   \    \           /    /__________\    \     |    | \  \ |   |
// |    |  \    \           |     |          |    |    \    \        /    /_____________\    \    |    |  \       |
// |    |   \    \          |     |          |    |     \    \     /    /                \    \   |    |   \      |
// |____|    \____\         |_____|          |____|      \____\  /____/                   \____\  |____|    \_____|
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.lang.*;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;

@TeleOp(name="FullTeleop", group="FullBot")

public class KyranTeleop extends OpMode {
    HardwareFullBot robot = new HardwareFullBot();
    boolean shooting = false;
    boolean autoAim = false;
    boolean squared = false;
    Integer secondSquare = 0;
    Integer shots = 0;
    boolean xVal = false;
     // Code to run ONCE when the driver hits INIT

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        robot.init(hardwareMap);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }


    //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
    }


    //Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {

        //Variables

        float turnPower = -gamepad1.right_stick_x; //Turn robot
        float forwardPower = -gamepad1.left_stick_y; //FOrward and Back
        float strafePower = gamepad1.left_stick_x; //Strafe
        //float diagonalPoser = ?; //Diagonal
        // /*
        telemetry.addData("leftx:", java.lang.Math.abs(gamepad1.left_stick_x));
        telemetry.addData("lefty:", java.lang.Math.abs(gamepad1.left_stick_y));
        telemetry.update();
        // */
        // Diagonal
        boolean diagonalY = (java.lang.Math.abs(gamepad1.left_stick_y) > 0.3 && java.lang.Math.abs(gamepad1.left_stick_y) < 0.8);
        boolean diagonalX = (java.lang.Math.abs(gamepad1.left_stick_x) > 0.3 && java.lang.Math.abs(gamepad1.left_stick_x) < 0.8);
        boolean diagonal = diagonalY && diagonalX;


        //Gamepad1

//if (!diagonal) {
        // Turning (Right stick X)
        robot.front_left.setPower(-turnPower);
        robot.front_right.setPower(turnPower);
        robot.back_left.setPower(-turnPower);
        robot.back_right.setPower(turnPower);

        // Forward (Left stick Y)
        robot.front_left.setPower(forwardPower);
        robot.front_right.setPower(forwardPower);
        robot.back_left.setPower(forwardPower);
        robot.back_right.setPower(forwardPower);

        // Strafe (Left stick X)
        robot.front_left.setPower(strafePower);
        robot.front_right.setPower(-strafePower);
        robot.back_left.setPower(-strafePower);
        robot.back_right.setPower(strafePower);
//}
/*
        if(diagonal) {
            //robot.front_left.setPower();
            robot.front_right.setPower(-1);
            robot.back_left.setPower(-1);
            //robot.back_right.setPower();
        }
*/


        // Arm Position Values

        // If arm starts up
        int downPosition = 1464;
        int upPosition = 0;
        int wallPosition = 675;

/*
        //If arm starts down
        int downPosition = 0;
        int upPosition = -1430;
        int wallPosition = -700;
*/
/*
        if (gamepad2.b) {
            robot.flick.setPosition(0.9);
        }
*/
        //Arm
        if (gamepad1.dpad_down) {
            robot.arm.setPower(-0.8);
            robot.arm.setTargetPosition(downPosition);
            robot.arm.setMode(RUN_TO_POSITION);
            sleep(1000);
            robot.claw.setPosition(0.0);
        } else if (gamepad1.dpad_up) {
            robot.claw.setPosition(1.0);
            sleep(1000);
            //robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.arm.setPower(0.8);
            robot.arm.setTargetPosition(upPosition);
            robot.arm.setMode(RUN_TO_POSITION);
        } else if (gamepad1.dpad_left || gamepad1.dpad_right) {
            robot.arm.setPower(-0.25);
            robot.arm.setTargetPosition(wallPosition);
            robot.arm.setMode(RUN_TO_POSITION);
            sleep(1000);
            robot.claw.setPosition(0);
        }

        //X button Override

        if (gamepad1.x) {
            robot.arm.setPower(0);
            robot.drop.setPosition(1);
            robot.flick.setPosition(0.38);
            robot.arm.setMode(RUN_WITHOUT_ENCODER);
            robot.claw.resetDeviceConfigurationForOpMode();
        }
        if (gamepad2.x || shots >= 3) {
            shooting = false;
        }

        //A button Drop Intake

        if (gamepad1.a) {
            robot.drop.setPosition(0);
        }


        //Gamepad2


        // intake power
        float intakePower = gamepad2.left_stick_y;
        robot.intake.setPower(intakePower);

        // Shooter Power
/*
        double shooterPower = (-gamepad2.right_stick_y);
        if (gamepad2.right_stick_button) {
            robot.shooter.setPower(shooterPower * 1.0);
        } else {
            robot.shooter.setPower(shooterPower * 0.75);
        }
*/

/*  Testing Velocity Shooting
        if (gamepad2.a) {
            shooting = true;
            //shoot(1340,1360,3);
            //shoot(1340,1360,3);
            //shoot(1340,1360,3);
        } else if (gamepad2.b) {
            shooting = false;
        }
 */
///*  //Testing AutoAim
        if (gamepad2.a) {
            autoAim = true;
            //shoot(1340,1360,3);
            //shoot(1340,1360,3);
            //shoot(1340,1360,3);
        } else if (gamepad2.b) {
            autoAim = false;
            shooting = false;
        }
 //*/


        // Flick Servo
       //Flick Servo range: 0.6 - 0.38
        if (gamepad2.left_bumper || shots >= 3) {
        //    robot.flick.setPosition(0.38);
            shooting = false;
        }
        if (gamepad2.right_bumper) {
                robot.flick.setPosition(0.38);
        }
        if (gamepad2.left_trigger > 0.2) {
        //        robot.flick.setPosition(0.6);
            shooting = true;
            }
        if (gamepad2.right_trigger > 0.2) {
                    robot.flick.setPosition(0.6);
        }

        double currentvelocity = robot.shooterEx.getVelocity();
        if (shooting) {
            robot.shooterEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.shooterEx.setMotorEnable();

            robot.shooterEx.setDirection(DcMotorEx.Direction.FORWARD);
            robot.shooterEx.setPower(1.0);
            robot.shooterEx.setVelocity(1335);  //1360
        if (currentvelocity > 1315 && currentvelocity < 1340) {  //1340, 1365
            robot.flick.setPosition(.60);
            sleep(500);
            robot.flick.setPosition(0.38);
            sleep(300);
            shots ++;
             }
        } else {
            robot.shooterEx.setPower(0.0);
            shots = 0;
          }

        // Auto Aim
        if (autoAim) {
            if(!xVal) {
                distanceStrafe();
            }
            if (xVal && secondSquare < 2 && autoAim) {
                teleopSquare();
            }
            if (squared && autoAim) {
                robot.front_left.setPower(-0.4);
                robot.back_left.setPower(-0.4);
                robot.front_right.setPower(-0.4);
                robot.back_right.setPower(-0.4);
                if (secondSquare < 1) {
                    sleep(200);
                } else {
                    sleep(650); //800
                }
                robot.front_left.setPower(0);
                robot.back_left.setPower(0);
                robot.front_right.setPower(0);
                robot.back_right.setPower(0);
                squared = false;
                secondSquare ++;
            }
            if (secondSquare >= 2 && autoAim) {
//                distanceStrafe();
  //          }
           // if (xVal && autoAim) {
                shooting = true;
                secondSquare = 0;
                autoAim = false;
                xVal = false;
            }

        }



        //telemetry.addData("Encoder", robot.arm.getCurrentPosition());
        telemetry.addData("autoAim?", autoAim);
        telemetry.addData("Current Velocity", currentvelocity);
        telemetry.update();


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.arm.setPower(0);
        robot.arm.setMode(RUN_WITHOUT_ENCODER);
        robot.claw.resetDeviceConfigurationForOpMode();

    }


    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
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


           // while (opModeIsActive() && i < shootCount) {

                // Setup a variable for each drive wheel to save power level for telemetry
                double currentvelocity = shooterMotor.getVelocity();
                 if (i==0){ sleep(500);} //700
                 else{ sleep(600); } //500
 //               if (currentvelocity == shootingSpeed) {

                    flick.setPosition(.60);
                    sleep(500);
                    flick.setPosition(0.38);
                    sleep(300);


                    // shooterMotor.setPower(0);

                    i++;
              //  }
                // Show the elapsed game time and wheel power.


            //}
            shooterMotor.setPower(0);


            //sleep(10000);
        }

        public void teleopSquare() {
            // NOTE: For Teleop only
            // For use in other OpModes, replace "if" with "while"

            boolean leftAligned = false;
            boolean rightAligned = false;

            if (robot.color_left.alpha() < 2000) {
                robot.front_left.setPower(0.45);
                robot.back_left.setPower(0.45);
            } else {
                robot.front_left.setPower(-0.2);
                robot.back_left.setPower(-0.2);
                leftAligned = true;
            }
            if (robot.color_right.alpha() < 1300) {
                robot.front_right.setPower(0.45);
                robot.back_right.setPower(0.45);
            } else {
                robot.front_right.setPower(-0.2);
                robot.back_right.setPower(-0.2);
                rightAligned = true;
            }
            if (leftAligned && rightAligned) {
                squared = true;
            }
        }
    public void distanceStrafe() {

        if (robot.distance_left.getDistance(DistanceUnit.CM) > 63) {
            robot.front_left.setPower(-0.6);
            robot.back_left.setPower(0.6);
            robot.front_right.setPower(0.7);
            robot.back_right.setPower(-0.6);
        } else {
            robot.front_left.setPower(0);
            robot.back_left.setPower(0);
            robot.front_right.setPower(0);
            robot.back_right.setPower(0);
            xVal = true;
        }
    }
}
