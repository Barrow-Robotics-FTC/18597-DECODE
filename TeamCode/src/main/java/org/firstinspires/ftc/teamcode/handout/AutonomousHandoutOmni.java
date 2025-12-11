package org.firstinspires.ftc.teamcode.handout; // TODO: CHANGE THIS TO YOUR CODE FOLDER

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * Barrow Robotics 18597 RoboClovers Delta - Autonomous Handout
 *
 * This is a simple autonomous that drives forward for a quarter second on a mecanum drivetrain.
 * It is intended to be a simple autonomous for teams that do not have an autonomous yet.
 * We have specifically designed this to work without interfering with our full autonomous programs.
 * Starting your robot in the correct position is very important to not interfere with us or the other teams.
 *
 * You will need to change the motor names and directions to match your configuration. See the TODO comments below.
 * Also change the package name to match your code folder. See the TODO comment above.
 *
 * Why should you use this?
 * This program will drive your robot off the starting line, earning our alliance 3 points.
 * This is better than doing nothing, which earns 0 points.
 *
 * How do you use this?
 * Start your robot on the audience wall (farthest from goals) facing our alliances human player zone (your robot should be sideways).
 * When the match starts, your robot will drive forward for a quarter second, then stop.
*/

@Autonomous(name="18597 Auto Handout", group="Autonomous")
public class AutonomousHandoutOmni extends LinearOpMode {
    @Override
    public void runOpMode() {
        // TODO: CHANGE THESE TO MATCH YOUR CONFIGURATION
        DcMotor frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        DcMotor frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        DcMotor backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        DcMotor backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        // TODO: CHANGE THESE TO MATCH YOUR CONFIGURATION
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Set motors to brake mode (so they stop quickly when power is set to zero)
        frontLeftDrive.setZeroPowerBehavior(BRAKE);
        frontRightDrive.setZeroPowerBehavior(BRAKE);
        backLeftDrive.setZeroPowerBehavior(BRAKE);
        backRightDrive.setZeroPowerBehavior(BRAKE);

        // Tell the driver that initialization is complete
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Move the robot to the robot forward at half power
        frontLeftDrive.setPower(0.5);
        frontRightDrive.setPower(0.5);
        backLeftDrive.setPower(0.5);
        backRightDrive.setPower(0.5);

        sleep(250); // Pause for a quarter second

        // Stop the robot
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
}
