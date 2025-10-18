package org.firstinspires.ftc.teamcode.tests;

// FTC SDK
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Custom launcher helper
import org.firstinspires.ftc.teamcode.utils.ManualLauncher;

/*
This is a simple test OpMode for the launcher (manual). It uses the custom manual Launcher helper (imported above).

To start the launcher, press the left bumper on gamepad 1. This will start the launcher speed up process.
If at any point, the launcher needs to be stopped, press the left bumper on gamepad 1 again, this will return the launcher to an idle state.
To command a launch, press the right bumper on gamepad 1. The launcher will only launch if it is up to speed and ready.
To adjust the target RPM of the launcher, use the dpad up and down buttons on gamepad 1.
*/

@TeleOp(name = "Manual Launcher Test", group = "Tests")
@SuppressWarnings("FieldCanBeLocal") // Suppress pointless Android Studio warnings
public class ManualLauncherTest extends LinearOpMode {
    private ManualLauncher launcher; // Custom launcher class
    private ManualLauncher.State launcherState; // Current launcher state

    @Override
    public void runOpMode() {
        // Create instance of launcher and initialize
        launcher = new ManualLauncher(hardwareMap, gamepad1);

        // Log completed initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for START to be pressed
        waitForStart();

        while (opModeIsActive()) {
            if (launcherState == ManualLauncher.State.IDLE) { // If the launcher is not running
                if (gamepad1.leftBumperWasPressed()) { // When right bumper is pressed
                    launcherState = launcher.update(); // Start the launcher (running update will get the launcher out of IDLE)
                }
            } else { // If the launcher is running
                launcherState = launcher.update(); // Run the launcher update loop
                if (gamepad1.leftBumperWasPressed()) { // When the right bumper is pressed
                    launcherState = launcher.stop(); // Stop the launcher (will move back to IDLE state)
                }
            }

            // If the right bumper is pressed, command a launch
            // Note: Conditions inside commandLaunch() will prevent launching if not ready
            if (gamepad1.rightBumperWasPressed()) {
                launcher.commandLaunch();
            }

            // Adjust target RPM with dpad up and down
            if (gamepad1.dpadUpWasPressed()) {
                launcher.setTargetRPM(launcher.getTargetRPM() + 25); // Increase target RPM by 25
            } else if (gamepad1.dpadDownWasPressed()) {
                launcher.setTargetRPM(launcher.getTargetRPM() - 25); // Decrease target RPM by 25
            }

            // Log status
            telemetry.addData("Launcher State", launcherState);
            telemetry.addData("Ready for Launch", launcher.isReadyForLaunch());
            telemetry.addData("Launches Completed", launcher.getLaunches());
            telemetry.addData("", "");
            telemetry.addData("Target RPM", launcher.getTargetRPM());
            telemetry.addData("Left Motor RPM", launcher.getLeftRPM());
            telemetry.addData("Right Motor RPM", launcher.getRightRPM());
            telemetry.addData("", "");
            telemetry.addData("Tapper Position (commanded)", launcher.getCommandedTapperPosition());
            telemetry.update();
        }
    }
}