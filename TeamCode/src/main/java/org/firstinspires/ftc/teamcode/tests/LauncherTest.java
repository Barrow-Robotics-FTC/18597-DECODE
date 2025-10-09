package org.firstinspires.ftc.teamcode.tests;

// FTC SDK
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Panels
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

// Local helper files
import org.firstinspires.ftc.teamcode.utils.Launcher;

/*
This is a simple test OpMode for the launcher. It uses the custom Launcher helper (imported above).

To start the launcher, press the right bumper on gamepad 1. This will run the launcher update loop to launch 3 artifacts.
If at any point, the launcher needs to be stopped, press the right bumper on gamepad 1 again, this will return the launcher to an idle state.
*/

@TeleOp(name = "Launcher Test", group = "Tests")
@Configurable // Use Panels
@SuppressWarnings("FieldCanBeLocal") // Suppress pointless Android Studio warnings
public class LauncherTest extends LinearOpMode {
    private TelemetryManager panelsTelemetry; // Panels telemetry
    private Launcher launcher; // Custom launcher class
    private Launcher.State launcherState; // Current launcher state

    @Override
    public void runOpMode() {
        // Initialize Panels telemetry
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Create instance of launcher and initialize
        launcher = new Launcher();
        launcher.init(hardwareMap);

        // Log completed initialization to Panels and driver station
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry); // Update Panels and driver station after logging

        // Wait for the TeleOp period to start (driver presses START)
        waitForStart();

        while (opModeIsActive()) {
            if (launcherState == Launcher.State.IDLE) { // If the launcher is not running
                if (gamepad1.right_bumper) { // When right bumper is pressed
                    launcherState = launcher.update(); // Start the launcher (running update will get the launcher out of IDLE)
                }
            } else { // If the launcher is running
                launcherState = launcher.update(); // Run the launcher update loop
                if (gamepad1.right_bumper) { // When the right bumper is pressed
                    launcher.stop(); // Stop the launcher (will move back to IDLE state)
                }
            }

            panelsTelemetry.debug("Launcher State", launcherState);
            panelsTelemetry.debug("Left Motor RPM", launcher.getLeftRPM());
            panelsTelemetry.debug("Right Motor RPM", launcher.getRightRPM());
            panelsTelemetry.debug("Tapper Rotation (commanded)", launcher.getCommandedTapperRotation());
            panelsTelemetry.update(telemetry); // Update Panels and Driver Station after logging
        }
    }
}