package org.firstinspires.ftc.teamcode.test;

// FTC SDK
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

// Local helper files
import org.firstinspires.ftc.teamcode.Constants.Mode;
import org.firstinspires.ftc.teamcode.Robot;

/*
Launcher Tuning OpMode
Tune the PDFT controllers for the launcher subsystem
Use the Panels graph to visualize the RPM response to changes in the constants
See tuning instructions in robot class

Controls:
Right Bumper: Toggle launcher speed up
D-Pad Up/Down: Increase/Decrease P
D-Pad Right/Left: Increase/Decrease D
Triangle/Cross: Increase/Decrease F
Circle/Square: Increase/Decrease T
*/

@TeleOp(name = "Launcher Tuner", group = "Tests")
@SuppressWarnings("FieldCanBeLocal") // Suppress pointless Android Studio warnings
public class LauncherTuner extends LinearOpMode {
    public boolean tuningLeft = true;
    private Robot robot; // Custom robot class
    private double p;
    private double d;
    private double f;
    private double t;

    private FilteredPIDFCoefficients getCoefficients() {
        if (tuningLeft) {
            return robot.launcher.getLeftControllerCoefficients();
        } else {
            return robot.launcher.getRightControllerCoefficients();
        }
    }

    @Override
    public void runOpMode() {
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Create instance of launcher and initialize
        robot = new Robot(hardwareMap, Mode.TELEOP);

        // Turn off the motor we aren't tuning
        if (tuningLeft) {
            robot.launcher.updateRightControllerCoefficients(new FilteredPIDFCoefficients(0, 0, 0, 0, 0));
        } else {
            robot.launcher.updateLeftControllerCoefficients(new FilteredPIDFCoefficients(0, 0, 0, 0, 0));
        }

        // Set initial coefficients from Constants
        p = getCoefficients().P;
        d = getCoefficients().D;
        f = getCoefficients().F;
        t = getCoefficients().T;

        // Log completed initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for START to be pressed
        waitForStart();

        while (opModeIsActive()) {
            // Update robot
            robot.update();

            // Tune with the gamepad
            if (gamepad1.dpad_up) { p += 0.001; }
            if (gamepad1.dpad_down) { p -= 0.001; }
            if (gamepad1.dpad_right) { d += 0.001; }
            if (gamepad1.dpad_left) { d -= 0.001; }
            if (gamepad1.triangle) { f += 0.001; }
            if (gamepad1.cross) { f -= 0.001; }
            if (gamepad1.circle) { t += 0.001; }
            if (gamepad1.square) { t -= 0.001; }

            // Update the controller coefficients
            FilteredPIDFCoefficients newCoeffs = new FilteredPIDFCoefficients(p, 0, d, f, t);
            if (tuningLeft) {
                robot.launcher.updateLeftControllerCoefficients(newCoeffs);
            } else {
                robot.launcher.updateRightControllerCoefficients(newCoeffs);
            }

            // Right Bumper: toggle launcher speed up
            if (gamepad1.rightBumperWasPressed()) {
                if (robot.launcher.isActive()) { // If launcher isn't idle
                    robot.launcher.stop(); // Stop the launcher
                } else {
                    robot.launcher.speedUp(); // Speed up the launcher
                }
            }

            // Panels telemetry for a graph
            telemetryM.addData("target", robot.launcher.getTargetRPM());
            telemetryM.addData("rpm", tuningLeft ? robot.launcher.getLeftRPM(robot) : robot.launcher.getRightRPM(robot));
            telemetryM.update();

            // Log status
            telemetry.addData("Launcher State", robot.launcher.getState());
            telemetry.addData("Target RPM", robot.launcher.getTargetRPM());
            telemetry.addData("Left Motor RPM", robot.launcher.getLeftRPM(robot));
            telemetry.addData("Right Motor RPM", robot.launcher.getRightRPM(robot));
            telemetry.update();
        }
    }
}