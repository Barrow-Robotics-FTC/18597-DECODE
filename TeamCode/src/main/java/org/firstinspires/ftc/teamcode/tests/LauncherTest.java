package org.firstinspires.ftc.teamcode.tests;

// FTC SDK
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Local helper files
import org.firstinspires.ftc.teamcode.utils.Launcher;

/*
This is a simple test OpMode for the launcher. It uses the custom Launcher helper (imported above).

To start the launcher, press the right bumper on gamepad 1. This will run the launcher update loop to launch 3 artifacts.
If at any point, the launcher needs to be stopped, press the right bumper on gamepad 1 again, this will return the launcher to an idle state.
To adjust the target RPM of the launcher, use the dpad up and down buttons on gamepad 1.
*/

@TeleOp(name = "Launcher Test", group = "Tests")
@SuppressWarnings("FieldCanBeLocal") // Suppress pointless Android Studio warnings
public class LauncherTest extends LinearOpMode {
    private Launcher launcher; // Custom launcher class
    private Launcher.State launcherState; // Current launcher state

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
            if (launcherState == Launcher.State.IDLE) { // If the launcher is not running
                if (gamepad1.rightBumperWasPressed()) { // When right bumper is pressed
                    launcherState = launcher.update(true); // Start the launcher (running update will get the launcher out of IDLE)
                }
            } else { // If the launcher is running
                launcherState = launcher.update(true); // Run the launcher update loop
                if (gamepad1.rightBumperWasPressed()) { // When the right bumper is pressed
                    launcherState = launcher.stop(); // Stop the launcher (will move back to IDLE state)
                }
            }

            if (gamepad1.dpadUpWasPressed()) {
                launcher.setTargetRPM(launcher.getTargetRPM() + 25); // Increase target RPM by 50
            } else if (gamepad1.dpadDownWasPressed()) {
                launcher.setTargetRPM(launcher.getTargetRPM() - 25); // Decrease target RPM by 50
            }

            // Log status
            telemetry.addData("Launcher State", launcherState);
            telemetry.addData("Launches Completed", launcher.getLaunches());
            telemetry.addData("Target RPM", launcher.getTargetRPM());
            telemetry.addData("Left Motor RPM", launcher.getLeftRPM());
            telemetry.addData("Right Motor RPM", launcher.getRightRPM());
            telemetry.addData("Tapper Rotation (commanded)", launcher.getCommandedTapperRotation());
            telemetry.update();
        }
    }
}