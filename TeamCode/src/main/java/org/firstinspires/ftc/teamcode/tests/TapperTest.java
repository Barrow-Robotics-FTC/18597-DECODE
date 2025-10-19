package org.firstinspires.ftc.teamcode.tests;

// FTC SDK
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

// launcher constants
import static org.firstinspires.ftc.teamcode.utils.Constants.LauncherConstants.TAPPER_PUSHED_POSITION;

/*
This is a simple test OpMode for the tapper servo.
To move the tapper, press the right bumper on gamepad 1. This will toggle the tapper between the pushed and retracted positions.
To adjust the pushed tapper position, use the dpad up and down buttons on gamepad 1.
*/

@TeleOp(name = "Launcher Test", group = "Tests")
@SuppressWarnings("FieldCanBeLocal") // Suppress pointless Android Studio warnings
public class TapperTest extends LinearOpMode {
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
                    tapper.setPosition(TAPPER_PUSHED_POSITION); // Push the tapper
                    tapperPositioned = true;
                } else {
                    tapper.setPosition(-TAPPER_PUSHED_POSITION); // Retract the tapper
                    tapperPositioned = false;
                }
            }

            if (gamepad1.dpadUpWasPressed()) {
                TAPPER_PUSHED_POSITION = TAPPER_PUSHED_POSITION + 0.01; // Increase tapper position by 0.01
                if (tapperPositioned) { tapper.setPosition(TAPPER_PUSHED_POSITION); } // If the tapper is currently pushed, update its position
            } else if (gamepad1.dpadDownWasPressed()) {
                TAPPER_PUSHED_POSITION = TAPPER_PUSHED_POSITION - 0.01; // Decrease tapper position by 0.01
                if (tapperPositioned) { tapper.setPosition(TAPPER_PUSHED_POSITION); } // If the tapper is currently pushed, update its position
            }

            // Log status
            telemetry.addData("Tapper Positioned", tapperPositioned);
            telemetry.addData("Target Tapper Position", TAPPER_PUSHED_POSITION);
            telemetry.addData("Tapper Rotation (commanded)", tapper.getPosition());
            telemetry.update();
        }
    }
}