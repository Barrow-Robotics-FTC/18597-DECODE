package org.firstinspires.ftc.teamcode.test;

// FTC SDK
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

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
*/

@TeleOp(name = "Launcher Tuner", group = "Tests")
@SuppressWarnings("FieldCanBeLocal") // Suppress pointless Android Studio warnings
public class LauncherTuner extends LinearOpMode {
    public boolean tuningLeft = true;
    private Robot robot; // Custom robot class
    private double p;
    private double d;
    private double f;

    private PIDFCoefficients getCoefficients() {
        if (tuningLeft) {
            return robot.launcher.getLeftControllerCoefficients(robot);
        } else {
            return robot.launcher.getRightControllerCoefficients(robot);
        }
    }

    @Override
    public void runOpMode() {
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Create instance of launcher and initialize
        robot = new Robot(hardwareMap, Mode.TELEOP);

        // Turn off the motor we aren't tuning
        if (tuningLeft) {
            robot.launcher.updateRightControllerCoefficients(robot, new PIDFCoefficients(0, 0, 0, 0));
        } else {
            robot.launcher.updateLeftControllerCoefficients(robot, new PIDFCoefficients(0, 0, 0, 0));
        }

        // Set initial coefficients from Constants
        p = getCoefficients().p;
        d = getCoefficients().d;
        f = getCoefficients().f;

        // Log completed initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for START to be pressed
        waitForStart();

        while (opModeIsActive()) {
            // Update robot
            robot.update();

            // Tune with the gamepad
            if (gamepad1.dpadUpWasPressed()) { p += 0.1; }
            if (gamepad1.dpadDownWasPressed()) { p -= 0.1; }
            if (gamepad1.dpadRightWasPressed()) { d += 0.1; }
            if (gamepad1.dpadLeftWasPressed()) { d -= 0.1; }
            if (gamepad1.triangleWasPressed()) { f += 0.1; }
            if (gamepad1.crossWasPressed()) { f -= 0.1; }

            // Update the controller coefficients
            PIDFCoefficients newCoeffs = new PIDFCoefficients(p, 0, d, f);
            if (tuningLeft) {
                robot.launcher.updateLeftControllerCoefficients(robot, newCoeffs);
            } else {
                robot.launcher.updateRightControllerCoefficients(robot, newCoeffs);
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
            telemetry.addData("P", newCoeffs.p);
            telemetry.addData("D", newCoeffs.d);
            telemetry.addData("F", newCoeffs.f);
            telemetry.update();
        }
    }
}