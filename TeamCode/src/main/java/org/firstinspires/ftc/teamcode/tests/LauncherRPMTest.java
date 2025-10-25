package org.firstinspires.ftc.teamcode.tests;

// FTC SDK
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

// Local helper files
import static org.firstinspires.ftc.teamcode.utils.Constants.LauncherConstants.*;

/*
This is a simple test OpMode to debug the launcher motor RPM.

To start the launcher, press the left bumper on gamepad 1. This will speed up the motors to the target RPM.
Press the left bumper again to stop the launcher.
*/

@TeleOp(name = "Launcher RPM Test", group = "Tests")
@SuppressWarnings("FieldCanBeLocal") // Suppress pointless Android Studio warnings
public class LauncherRPMTest extends LinearOpMode {
    private DcMotorEx leftMotor; // Left flywheel motor (looking from the robots perspective)
    private DcMotorEx rightMotor; // Right flywheel motor (looking from the robots perspective)
    private boolean launcherRunning = false; // True when launcher is running

    // Helper function to stop motors and reset servos
    private void resetMotors() {
        leftMotor.setVelocity(0);
        rightMotor.setVelocity(0);
    }

    @Override
    public void runOpMode() {
        // initialize hardware
        leftMotor = hardwareMap.get(DcMotorEx.class, "launcher_left");
        rightMotor = hardwareMap.get(DcMotorEx.class, "launcher_right");

        // Set motor directions and characteristics
        rightMotor.setDirection(REVERSE); // Reverse right motor
        rightMotor.setZeroPowerBehavior(BRAKE);
        rightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, LAUNCHER_PIDF_COEFFICIENTS);
        leftMotor.setZeroPowerBehavior(BRAKE);
        leftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, LAUNCHER_PIDF_COEFFICIENTS);

        // Reset motors and servos to default state
        resetMotors();

        // Log completed initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for START to be pressed
        waitForStart();

        while (opModeIsActive()) {
            // If the left bumper is pressed
            if (gamepad1.leftBumperWasPressed()) {
                if (!launcherRunning) {
                    // Start launcher
                    leftMotor.setVelocity(TARGET_RPM);
                    rightMotor.setVelocity(TARGET_RPM);
                    launcherRunning = true;
                } else {
                    // Stop launcher
                    resetMotors();
                    launcherRunning = false;
                }
            }

            // Create variables to check if each motor is within the RPM tolerance
            boolean leftInTol = Math.abs(TARGET_RPM - leftMotor.getVelocity()) <= RPM_TOLERANCE;
            boolean rightInTol = Math.abs(TARGET_RPM - rightMotor.getVelocity()) <= RPM_TOLERANCE;

            // Log status
            telemetry.addData("Target RPM", TARGET_RPM);
            telemetry.addData("Left Motor RPM", leftMotor.getVelocity());
            telemetry.addData("Right Motor RPM", rightMotor.getVelocity());
            telemetry.addData("Left Motor In Tolerance", leftInTol);
            telemetry.addData("Right Motor In Tolerance", rightInTol);
            telemetry.addData("Ready for Launch", leftInTol && rightInTol);
            telemetry.update();
        }
    }
}