package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants.Mode;
import org.firstinspires.ftc.teamcode.Robot;

/*
Intake Test OpMode
Full test of the intake subsystem

Controls:
Left Bumper: Toggle intake
*/

@TeleOp(name = "Intake Test", group = "Tests")
@SuppressWarnings("FieldCanBeLocal") // Suppress pointless Android Studio warnings
public class IntakeTest extends LinearOpMode {
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

            // Log status
            telemetry.addData("Intake State", robot.intake.getState());
            telemetry.update();
        }
    }
}