package org.firstinspires.ftc.teamcode.tests;

// FTC SDK
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Local helper files
import org.firstinspires.ftc.teamcode.utils.Launcher;
import org.firstinspires.ftc.teamcode.utils.Constants.LauncherConstants.LauncherState;
import org.firstinspires.ftc.teamcode.utils.Constants.LauncherConstants.LauncherReturnProps;

/*
This is a simple test OpMode for the launcher. It uses the custom Launcher helper (imported above).

To start the launcher, press the left bumper on gamepad 1. This will get the launcher up to the target RPM..
Press the left bumper again to stop the launcher.
To launch artifacts, press the left trigger to launch 1 artifact, or the right trigger to launch 3 artifacts.
NOTE: You can launch without speeding up, it will be sped up automatically and then it will shut back down.
*/

@TeleOp(name = "Launcher Test", group = "Tests")
@SuppressWarnings("FieldCanBeLocal") // Suppress pointless Android Studio warnings
public class LauncherTest extends LinearOpMode {
    private Launcher launcher; // Custom launcher class
    private LauncherReturnProps launcherStatus; // Current launcher state
    private boolean launcherIsActive = false; // True when launcher is active (speed up)
    private boolean launcherIsLaunching = false; // True when launcher is launching
    private final ElapsedTime launchCycleTimer = new ElapsedTime(); // Keeps track of the time since the last completed launch cycle

    @Override
    public void runOpMode() {
        // Create instance of launcher and initialize
        launcher = new Launcher(hardwareMap);

        // Log completed initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for START to be pressed
        waitForStart();

        while (opModeIsActive()) {
            // If the left bumper is pressed and the launcher is idle
            if (gamepad1.leftBumperWasPressed()) {
                if (launcherIsActive) {
                    launcher.stop(); // Stop the launcher
                    launcherIsActive = false; // Set active flag to false
                } else {
                    launcher.speedUp(); // Speed up the launcher
                    launcherIsActive = true; // Set active flag to true
                }
            }

            // If the launcher is running, we don't want to command more
            if (!launcherIsLaunching) {
                // If the left trigger is pressed, launch an artifact
                if (gamepad1.left_trigger > 0.5) {
                    launcher.launch(1); // Command the launcher to launch 1 artifact
                    launcherIsActive = true; // Set active flag to true (may already be true)
                    launcherIsLaunching = true; // Set launching flag to true
                }

                // If the right trigger is pressed, launch 3 artifacts
                if (gamepad1.right_trigger > 0.5) {
                    launcher.launch(3); // Command the launcher to launch 3 artifacts
                    launcherIsActive = true; // Set active flag to true (may already be true)
                    launcherIsLaunching = true; // Set launching flag to true
                }
            }

            // Update launcher
            launcherStatus = launcher.update();

            // Check if a launch cycle was completed
            if (launcherStatus.cycleCompleted) {
                launchCycleTimer.reset(); // Reset launch cycle timer
                launcherIsLaunching = false; // Reset launching flag

                if (launcherStatus.state == LauncherState.IDLE) {
                    launcherIsActive = false; // If the launcher was stopped after launching, set active flag to false
                }
            }

            // Log status
            telemetry.addData("Launcher State", launcherStatus.state);
            telemetry.addData("Launches Completed", launcher.getLaunches());
            telemetry.addData("Target RPM", launcher.getTargetRPM());
            telemetry.addData("Left Motor RPM", launcher.getLeftRPM());
            telemetry.addData("Right Motor RPM", launcher.getRightRPM());
            telemetry.addData("Tapper Position (commanded)", launcher.getCommandedTapperPosition());
            telemetry.update();
        }
    }
}