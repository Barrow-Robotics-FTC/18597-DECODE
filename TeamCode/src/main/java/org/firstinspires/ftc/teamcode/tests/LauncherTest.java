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

To start the launcher, press the right bumper on gamepad 1. This will run the launcher update loop to launch 3 artifacts.
If at any point, the launcher needs to be stopped, press the right bumper on gamepad 1 again, this will return the launcher to an idle state.
To adjust the target RPM of the launcher, use the dpad up and down buttons on gamepad 1.
*/

@TeleOp(name = "Launcher Test", group = "Tests")
@SuppressWarnings("FieldCanBeLocal") // Suppress pointless Android Studio warnings
public class LauncherTest extends LinearOpMode {
    private Launcher launcher; // Custom launcher class
    private LauncherReturnProps launcherStatus; // Current launcher state
    private ElapsedTime launchCycleTimer = new ElapsedTime(); // Keeps track of the time since the last completed launch cycle

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
            if (gamepad1.leftBumperWasPressed() && launcherStatus.state == LauncherState.IDLE) {
                launcher.speedUp(); // Speed up the launcher
            }

            // If the right bumper is pressed
            if (gamepad1.rightBumperWasPressed()) {
                launcher.launch(1); // Launch an artifact
            }

            // Use DPad buttons to control launcher speed
            if (gamepad1.dpadUpWasPressed()) {
                launcher.setTargetRPM(launcher.getTargetRPM() + 25); // Increase target RPM by 25
            } else if (gamepad1.dpadDownWasPressed()) {
                launcher.setTargetRPM(launcher.getTargetRPM() - 25); // Decrease target RPM by 25
            }

            // Update launcher
            launcherStatus = launcher.update();

            // Check if a launch cycle was completed
            if (launcherStatus.cycleCompleted) {
                launchCycleTimer.reset();
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