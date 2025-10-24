package org.firstinspires.ftc.teamcode.teleop;

// FTC SDK
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

// Local helper files
import static org.firstinspires.ftc.teamcode.utils.Constants.TeleOp.*;
import org.firstinspires.ftc.teamcode.utils.Constants.LauncherConstants.LauncherReturnProps;
import org.firstinspires.ftc.teamcode.utils.Constants.LauncherConstants.LauncherState;
import org.firstinspires.ftc.teamcode.utils.Launcher;

/*
Gamepad Map for LM1 no Pedro Pathing
The FTCPadMap file and image available in /TeamCode/src/main/gamepadMaps/lm1.ftcpadmap and /TeamCode/src/main/gamepadMaps/lm1.png
You can't view in Android Studio unless you switch from "Android" to "Project" view in the left side file viewer, then navigate to the file
Upload the file to https://barrow-robotics-ftc.github.io/FTCPadMap/ for an interactive view

Drive Coach: NAME
Human Player: NAME
Gamepad 1 (Driver): NAME
    Left Stick X: Robot translation movement
    Left Stick Y: Robot axial movement

    Right Stick X: Robot rotational movement
    DPad Right: Toggle slow mode
    Right Bumper: Toggle launcher speed up (will hold speed once sped up until you press this again)
    Left Trigger: Launch 1 artifact (position will be held automatically until launch completes)
    Right Trigger: Launch 3 artifacts (position will be held automatically until launch completes)
 */

@TeleOp(name = "LM1 TeleOp (No Pedro)", group = "TeleOp")
@SuppressWarnings("FieldCanBeLocal") // Suppress pointless Android Studio warnings
public class LM1noPedroTeleOp extends LinearOpMode {

    // Driver controller variables
    private boolean slowMode = false;

    // Other variables
    private final ElapsedTime runtime = new ElapsedTime();
    private LauncherReturnProps launcherStatus; // Current launcher state
    private Launcher launcher; // Custom launcher class
    private boolean launcherIsLaunching = false; // Is the launcher currently launching?

    }

    @Override
    public void runOpMode() {
        // Initialize all utilities used in TeleOp
        launcher = new Launcher(hardwareMap);

        // Log completed initialization to Panels and driver station
        telemetry.addData("Status", "Initialized");
        telemetry.update(); // Update Panels and driver station after logging

        // Wait for the TeleOp period to start (driver presses START)
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // Gamepad 1 DPad Right: toggle slow mode
            if (gamepad1.dpadRightWasPressed()) {
                slowMode = !slowMode;
            }

            // Gamepad 1 Right Bumper: Toggle launcher speed up
            if (gamepad1.rightBumperWasPressed() && launcherStatus.state == LauncherState.IDLE) {
                launcher.speedUp(); // Speed up the launcher
            }

            // Gamepad 1 Left Trigger: Launch 1 artifact
            if (gamepad1.right_trigger > 0.5 && !launcherIsLaunching) {
                // Note: If the launch command will call speedUp() if the launcher is idle, so no need to check here
                launcher.launch(1); // Start the launch of 1 artifact
                launcherIsLaunching = true; // Indicate that the launcher is launching
            }

            // Gamepad 1 Right Trigger: Launch 3 artifacts
            if (gamepad1.right_trigger > 0.5 && !launcherIsLaunching) {
                // Note: If the launch command will call speedUp() if the launcher is idle, so no need to check here
                launcher.launch(3); // Start the launch of 3 artifacts
                launcherIsLaunching = true; // Indicate that the launcher is launching
            }

            // Update launcher (nothing will happen when launcher is idle)
            launcherStatus = launcher.update();
            if (launcherStatus.cycleCompleted) { // If a launch cycle has complete
                launcherIsLaunching = false; // Indicate that the launcher is no longer launching
            }

            // Log status
            telemetry.addData("Run Time: ", runtime.seconds());
            telemetry.addData("Automated Drive: ", automatedDrive);
            telemetry.addData("Launcher State: ", launcherStatus.state);
            telemetry.update();

            // No intake for meet one, so this code may be referenced in LM2
            /*
            // When Left trigger is pressed: Initialize / Disable Intake
            if (gamepad2.left_trigger > 0.1) { // When right bumper is pressed
                intakeRunning = !intakeRunning; // Toggle intake state
                if (intakeRunning) {
                    intake.run(); // Start the intake
                }
                else {
                    intake.stop(); // Stop the intake
                }
            }
            */
        }
    }
}