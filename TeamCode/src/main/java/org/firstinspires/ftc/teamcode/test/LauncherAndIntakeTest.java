package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants.Mode;
import org.firstinspires.ftc.teamcode.Robot;

/*
Launcher and Intake Test OpMode
Full test of the launcher subsystem and intake subsystem

Controls:
Left Bumper: Toggle intake
Right Bumper: Toggle launcher speed up
Left Trigger: Launch 1 artifact
Right Trigger: Launch 3 artifacts
*/

@TeleOp(name = "Launcher Test", group = "Tests")
@SuppressWarnings("FieldCanBeLocal") // Suppress pointless Android Studio warnings
public class LauncherAndIntakeTest extends LinearOpMode {
    private Robot robot; // Custom robot class

    @Override
    public void runOpMode() {
        // Create instance of launcher and initialize
        robot = new Robot(hardwareMap, Mode.TELEOP);

        // Log completed initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for START to be pressed
        waitForStart();

        while (opModeIsActive()) {
            // Update robot
            robot.update(gamepad1, gamepad2);

            // Left Bumper: toggle intake
            if (gamepad1.leftBumperWasPressed()) {
                if (robot.intake.isActive()) { // If intake isn't idle
                    robot.intake.stop(); // Stop the intake
                } else {
                    robot.intake.run(); // Start the intake
                }
            }

            // Right Bumper: toggle launcher speed up
            if (gamepad1.rightBumperWasPressed()) {
                if (robot.launcher.isActive()) { // If launcher isn't idle
                    robot.launcher.stop(); // Stop the launcher
                } else {
                    robot.launcher.speedUp(true); // Speed up the launcher
                }
            }

            // Check if the launcher isn't currently launching
            if (!robot.launcher.isLaunching()) {
                // Left Trigger: Launcher 1 artifact
                if (robot.gamepad1LeftTriggerPressed(gamepad1)) {
                    robot.launcher.launch(1); // Command the launcher to launch 1 artifact
                }

                // Right Trigger: Launch 3 artifacts
                if (robot.gamepad1RightTriggerPressed(gamepad1)) {
                    robot.launcher.launch(3); // Command the launcher to launch 3 artifacts
                }
            }

            // Log status
            telemetry.addData("Intake State", robot.intake.getState());
            telemetry.addData("Launcher State", robot.launcher.getState());
            telemetry.addData("Tapper State", robot.tapper.getState());
            telemetry.addData("Target RPM", robot.launcher.getTargetRPM());
            telemetry.addData("Left Motor RPM", robot.launcher.getLeftRPM(robot));
            telemetry.addData("Right Motor RPM", robot.launcher.getRightRPM(robot));
            telemetry.update();
        }
    }
}