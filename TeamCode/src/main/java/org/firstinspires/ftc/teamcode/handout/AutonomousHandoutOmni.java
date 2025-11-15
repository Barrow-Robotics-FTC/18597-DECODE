package org.firstinspires.ftc.teamcode.handout; // TODO: CHANGE THIS TO YOUR CODE FOLDER

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Barrow Robotics 18597 RoboClovers Delta - Omni Drive Autonomous Handout
 *
 * This is a simple autonomous that drives forward for 1 second using omni drive.
 * It is intended to be a simple autonomous for teams that do not have an autonomous yet.
 *
 * You will need to change the motor names and directions to match your configuration. See the TODO comments below.
 * Also change the package name to match your code folder. See the TODO comment above.
 *
 * Why should you use this?
 * This program will drive your robot off the starting line, earning our alliance 3 points.
 * This is better than doing nothing, which earns 0 points.
 *
 * How do you use this?
 * Start your robot on the audience wall facing forwards on the far edge of the launch line.
 * When the match starts, your robot will drive forward for 1 second, then stop.
*/

@Autonomous(name="18597 Omni Auto Handout", group="Autonomous")
public class AutonomousHandoutOmni extends LinearOpMode {
    // Timer to track how long the robot has been moving
    private final ElapsedTime robotMovingFor = new ElapsedTime();

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

        // Set motors to brake mode
        frontLeftDrive.setZeroPowerBehavior(BRAKE);
        frontRightDrive.setZeroPowerBehavior(BRAKE);
        backLeftDrive.setZeroPowerBehavior(BRAKE);
        backRightDrive.setZeroPowerBehavior(BRAKE);

        // Tell the driver that initialization is complete
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        robotMovingFor.reset(); // Reset the robot move timer to zero

        while (opModeIsActive()) {
            // Move the robot to the robot forward at half power
            frontLeftDrive.setPower(0.5);
            frontRightDrive.setPower(0.5);
            backLeftDrive.setPower(0.5);
            backRightDrive.setPower(0.5);

            // If the robot has been moving for 1 second, stop the robot
            if (robotMovingFor.seconds() >= 1) {
                frontLeftDrive.setPower(0);
                frontRightDrive.setPower(0);
                backLeftDrive.setPower(0);
                backRightDrive.setPower(0);
                break; // Exit the loop (end OpMode)
            }
        }
    }
}
