package org.firstinspires.ftc.teamcode.tests;

// FTC SDK
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Local helper files
import org.firstinspires.ftc.teamcode.utils.Intake;

/*
This is a simple test OpMode for the intake. It uses the custom Intake helper (imported above).

Start the intake with the left bumper on gamepad 1. This will run the intake until the left bumper is pressed again.
*/

@TeleOp(name = "Intake Test", group = "Tests")
@SuppressWarnings("FieldCanBeLocal") // Suppress pointless Android Studio warnings
public class IntakeTest extends LinearOpMode {
    private Intake intake; // Custom intake class
    private boolean intakeRunning = false; // True when intake is running

    @Override
    public void runOpMode() {
        // Create instance of intake and initialize
        intake = new Intake(hardwareMap);

        // Log completed initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for START to be pressed
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.leftBumperWasPressed()) { // When right bumper is pressed
                intakeRunning = !intakeRunning; // Toggle intake state
                if (intakeRunning) {
                    intake.run(); // Start the intake
                } else {
                    intake.stop(); // Stop the intake
                }
            }

            telemetry.addData("Intake Running", intakeRunning);
            telemetry.update(); // Update Panels and Driver Station after logging
        }
    }
}