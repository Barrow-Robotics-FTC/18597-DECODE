package org.firstinspires.ftc.teamcode.tests;

// FTC SDK
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/*
This is a simple test OpMode for the launcher. It uses the custom Launcher helper (imported above).

To start the launcher, press the right bumper on gamepad 1. This will run the launcher update loop to launch 3 artifacts.
If at any point, the launcher needs to be stopped, press the right bumper on gamepad 1 again, this will return the launcher to an idle state.
To adjust the target RPM of the launcher, use the dpad up and down buttons on gamepad 1.
*/

@TeleOp(name = "Launcher Test", group = "Tests")
@SuppressWarnings("FieldCanBeLocal") // Suppress pointless Android Studio warnings
public class TapperTest extends LinearOpMode {
    // Editable variables
    private double TAPPER_ROTATION_AMOUNT = 0.5; // How much the tapper servo rotates to push a ball into the shooter

    // Other variables
    private Servo tapper; // Tapper servo
    private boolean tapperPositioned = false; // Whether the tapper is in the pushed position

    @Override
    public void runOpMode() {
        // Get the tapper servo from hardware map
        tapper = hardwareMap.get(Servo.class, "tapper");

        // Log completed initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for START to be pressed
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.rightBumperWasPressed()) { // When right bumper is pressed
                if (!tapperPositioned) {
                    tapper.setPosition(TAPPER_ROTATION_AMOUNT); // Push the tapper
                    tapperPositioned = true;
                } else {
                    tapper.setPosition(-TAPPER_ROTATION_AMOUNT); // Retract the tapper
                    tapperPositioned = false;
                }
            }

            if (gamepad1.dpadUpWasPressed()) {
                TAPPER_ROTATION_AMOUNT = TAPPER_ROTATION_AMOUNT + 0.01; // Increase tapper position by 0.01
                if (tapperPositioned) { tapper.setPosition(TAPPER_ROTATION_AMOUNT); } // If the tapper is currently pushed, update its position
            } else if (gamepad1.dpadDownWasPressed()) {
                TAPPER_ROTATION_AMOUNT = TAPPER_ROTATION_AMOUNT - 0.01; // Decrease tapper position by 0.01
                if (tapperPositioned) { tapper.setPosition(TAPPER_ROTATION_AMOUNT); } // If the tapper is currently pushed, update its position
            }

            // Log status
            telemetry.addData("Tapper Positioned", tapperPositioned);
            telemetry.addData("Target Tapper Position", TAPPER_ROTATION_AMOUNT);
            telemetry.addData("Tapper Rotation (commanded)", tapper.getPosition());
            telemetry.update();
        }
    }
}