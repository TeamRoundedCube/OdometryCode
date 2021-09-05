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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;

@Autonomous(name="KyranAuto", group="FullBot")

public class KyranAuto extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    HardwareFullBot robot = new HardwareFullBot();   // Use a Pushbot's hardware


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


        // Wait for the game to start (driver presses PLAY)
        clawClose();
        turnOnFlicker(0.38);
        //  turnOnFlicker(0.5);
        telemetry.addData("running", "running");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            shootGoal();
            DriveforTime(1, 1.75, 20);
            dropOffWobbleGoal();
        }

        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
    public void dropOffWobbleGoal() {
        moveArmDown(-50.0f);
        clawOpen();
        sleep(100);
        moveArmUp(92.0f);
    }
    public void clawOpen() {
        clawControl(0);
    }
    public void clawClose() {
        clawControl(1);
    }
    public void clawControl(double position) {

        robot.claw.setPosition(position);
        sleep(2000);

    }
    //Function that makes the arm move downwards
    public void moveArmDown(double angle) {

        robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.arm.setPower(0.6);
        sleep(600);

        robot.arm.setPower(0);

        runtime.reset();

        telemetry.update();
    }

    //Function that moves arm up
    public void moveArmUp(double angle) {
        robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.arm.setPower(-0.8);
        sleep(500);

        robot.arm.setPower(0);

        runtime.reset();

    } //After the motors have slept for 1000 miliseconds, the arm's finished position will be the most recent position and will update frequently
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

    public void turnOnShootingMotor() {

        robot.shooter.setPower(1.0); //instructing the amount of power the shooter must use to shoot
        // SLEEP VALUE IS VARIABLE BASED ON BATTERY LEVEL!!
        // Tested sleep values:
        // Full Battery: 300 (milliseconds)
        //
        sleep(300); // wait for motor to get good speed.

    }

    public void turnOffShootingMotor() {
        robot.shooter.setPower(0);
    } //switching power to 0 to stop movement

    public void turnOnFlicker(Double position) {
        robot.flick.setPosition(position); //moves flicker at high speed to a certain location
    }

    public void turnOffFlicker() {} //bringing flicker back to starting position
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
}