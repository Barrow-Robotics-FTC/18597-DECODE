package org.firstinspires.ftc.teamcode.tests;

// FTC SDK
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Local helper files
import org.firstinspires.ftc.teamcode.utils.AprilTag;
import org.firstinspires.ftc.teamcode.utils.Constants.Pattern;

/*
This is a simple test OpMode for April Tag pattern detection. It uses the custom AprilTag helper (imported above).
To detect the pattern, press the right bumper on gamepad 1. The detected pattern will be displayed on the driver station.
*/

@TeleOp(name = "Pattern Detection Test", group = "Tests")
@SuppressWarnings("FieldCanBeLocal") // Suppress pointless Android Studio warnings
public class PatternDetectionTest extends LinearOpMode {
    private AprilTag aprilTag; // Custom April Tag class
    private Pattern detectedPattern = null; // Last detected pattern

    @Override
    public void runOpMode() {
        // Create instance of intake and initialize
        aprilTag = new AprilTag(hardwareMap);

        // Log completed initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for START to be pressed
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.rightBumperWasPressed()) { // When right bumper is pressed
                detectedPattern = aprilTag.detectPattern(); // Detect the pattern
            }

            telemetry.addData("Detected Pattern", detectedPattern == null ? "None" : detectedPattern.toString());
            telemetry.update(); // Update Panels and Driver Station after logging
        }
    }
}