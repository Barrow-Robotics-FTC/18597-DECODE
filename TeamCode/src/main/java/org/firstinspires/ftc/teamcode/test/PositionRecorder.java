package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot;

/*
Position Recorder OpMode
Displays the current position of the robot in telemetry to help record positions for autonomous paths
Start the robot in the blue alliance human player zone corner (localize position)
*/

@TeleOp(name = "Position Recorder", group = "Tests")
@SuppressWarnings("FieldCanBeLocal") // Suppress pointless Android Studio warnings
public class PositionRecorder extends LinearOpMode {
    private Robot robot; // Custom robot class

    @Override
    public void runOpMode() {
        // Create instance of launcher and initialize
        robot = new Robot(hardwareMap, Constants.Mode.TELEOP);

        // Build poses and set start position to blue goal pose
        robot.buildPoses(Constants.Alliance.BLUE);
        robot.drivetrain.setStartingPose(robot.poses.localize);

        // Log completed initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for START to be pressed
        waitForStart();

        while (opModeIsActive()) {
            // Update robot
            robot.update(gamepad1, gamepad2);

            // Log status
            telemetry.addData("X", robot.drivetrain.getPose().getX());
            telemetry.addData("Y", robot.drivetrain.getPose().getY());
            telemetry.addData("Heading", Math.toDegrees(robot.drivetrain.getPose().getHeading()));
            telemetry.update();
        }
    }
}