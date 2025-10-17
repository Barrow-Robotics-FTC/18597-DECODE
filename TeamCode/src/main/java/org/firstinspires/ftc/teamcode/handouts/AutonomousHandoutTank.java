package org.firstinspires.ftc.teamcode.handouts; // TODO: CHANGE THIS TO YOUR CODE FOLDER

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * Barrow Robotics 18597 RoboClovers Delta - Tank Drive Autonomous Handout
 *
 * This is a simple autonomous that drives forward for 2 seconds using tank drive.
 * It is intended to be a simple autonomous for teams that do not have an autonomous yet.
 *
 * You will need to change the motor names and directions to match your configuration. See the TODO comments below.
 * Also change the package name to match your code folder. See the TODO comment above.
 *
 * Why should you use this?
 * This program will drive your robot off the starting line, earning our alliance 3 points.
 * Combined with our autonomous and endgame full + partial parking, we will earn a ranking point
 * So single handedly, this program will earn both of us a ranking point!
*/

@Autonomous(name="simple forward auto", group="Autonomous")
public class AutonomousHandoutTank extends LinearOpMode {
    // Timer to track how long the robot has been moving
    private final ElapsedTime robotMovingFor = new ElapsedTime();

    @Override
    public void runOpMode() {
        // TODO: CHANGE THESE TO MATCH YOUR CONFIGURATION
        DcMotor leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        DcMotor rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // TODO: CHANGE THESE TO MATCH YOUR CONFIGURATION
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Tell the driver that initialization is complete
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        robotMovingFor.reset(); // Reset the robot move timer to zero

        while (opModeIsActive()) {
            // Drive forward at half power
            leftDrive.setPower(.5);
            rightDrive.setPower(.5);

            // If the robot has been moving for 2 seconds, stop the robot
            if (robotMovingFor.seconds() >= 2.0) {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                break; // Exit the loop (end OpMode)
            }
        }
    }
}
