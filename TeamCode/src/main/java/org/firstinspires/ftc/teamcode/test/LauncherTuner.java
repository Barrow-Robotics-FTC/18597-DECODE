package org.firstinspires.ftc.teamcode.test;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import static org.firstinspires.ftc.teamcode.Constants.PVSCoefficients;
import org.firstinspires.ftc.teamcode.Constants.Mode;
import org.firstinspires.ftc.teamcode.Robot;

/*
Launcher Tuning OpMode
Tune the PVS controllers for the launcher subsystem
Use the Panels graph to visualize the RPM response to changes in the constants

Constants:
kV: Velocity feedforward
kP: Gain
kS: Static feedforward (unit conversion from velocity to power)

Tuning:
- Increase kS until the flywheel moves, back off until it stops
    - Like Pedro Pathing F
- Increase kV until the flywheel reaches target speed
- Tune kP to make it accelerate to the target speed quickly without overshooting

Controls:
Right Bumper: Toggle launcher speed up
*/
/*
@TeleOp(name = "Launcher Tuner", group = "Tests")
@SuppressWarnings("FieldCanBeLocal") // Suppress pointless Android Studio warnings
@Configurable
public class LauncherTuner extends LinearOpMode {
    public boolean tuningLeft = true;
    private Robot robot; // Custom robot class
    private double p;
    private double v;
    private double s;

    private PVSCoefficients getCoefficients(Robot robot) {
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
            robot.launcher.updateRightControllerCoefficients(robot, new PVSCoefficients(0, 0, 0));
        } else {
            robot.launcher.updateLeftControllerCoefficients(robot, new PVSCoefficients(0, 0, 0));
        }

        // Set initial coefficients from Constants
        p = getCoefficients(robot).p;
        v = getCoefficients(robot).v;
        s = getCoefficients(robot).s;

        // Log completed initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for START to be pressed
        waitForStart();

        while (opModeIsActive()) {
            // Update robot
            robot.update(gamepad1, gamepad2);

            // Update the controller coefficients
            PVSCoefficients newCoeffs = new PVSCoefficients(p, v, s);
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
                    robot.launcher.speedUp(true); // Speed up the launcher
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
            telemetry.addData("V", newCoeffs.v);
            telemetry.addData("S", newCoeffs.s);
            telemetry.update();
        }
    }
}
*/

/*
Launcher Tuning OpMode
Tune the PIDF controllers for the launcher subsystem
Use the Panels graph to visualize the RPM response to changes in the constants

Controls:
Right Bumper: Toggle launcher speed up
*/

@TeleOp(name = "Launcher Tuner", group = "Tests")
@SuppressWarnings("FieldCanBeLocal") // Suppress pointless Android Studio warnings
@Configurable
public class LauncherTuner extends LinearOpMode {
    public boolean tuningLeft = true;
    private Robot robot; // Custom robot class
    private double p;
    private double i;
    private double d;
    private double f;
    private boolean highStepToggle = true;
    private double highStep = 1;
    private double lowStep = 0.01;

    private PIDFCoefficients getCoefficients(Robot robot) {
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
        p = getCoefficients(robot).p;
        i = getCoefficients(robot).i;
        d = getCoefficients(robot).d;
        f = getCoefficients(robot).f;

        // Log completed initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for START to be pressed
        waitForStart();

        while (opModeIsActive()) {
            // Update robot
            robot.update(gamepad1, gamepad2);

            // Update the controller coefficients
            PIDFCoefficients newCoeffs = new PIDFCoefficients(p, i, d, f);
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
                    robot.launcher.speedUp(true); // Speed up the launcher
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
            telemetry.addData("I", newCoeffs.i);
            telemetry.addData("D", newCoeffs.d);
            telemetry.addData("F", newCoeffs.f);
            telemetry.update();
        }
    }
}