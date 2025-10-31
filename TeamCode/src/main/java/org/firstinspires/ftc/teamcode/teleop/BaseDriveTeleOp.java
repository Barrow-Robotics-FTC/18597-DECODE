package org.firstinspires.ftc.teamcode.teleop;

// FTC SDK
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Pedro Pathing
import com.pedropathing.follower.Follower;

// Local helpers
import static org.firstinspires.ftc.teamcode.utils.Constants.TeleOp.*;
import org.firstinspires.ftc.teamcode.utils.Constants;

@TeleOp(name="Base Drive TeleOp", group="TeleOp")
@SuppressWarnings("FieldCanBeLocal") // Suppress pointless Android Studio warnings
public class BaseDriveTeleOp extends LinearOpMode {
    // Pedro Pathing follower for TeleOp control
    private Follower follower;

    // Slow mode
    private boolean slowMode = false;

    @Override
    public void runOpMode() {
        // Initialize Pedro Pathing
        follower = Constants.Pedro.createFollower(hardwareMap);
        follower.update();

        // Log completed initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Start Pedro Pathing with TeleOp (manual) drive
        follower.startTeleopDrive(BRAKE_MODE);

        while (opModeIsActive()) {
            // Update follower and set drive power based on gamepad input
            follower.update();
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * (slowMode ? SLOW_MODE_MULTIPLIER : NORMAL_SPEED_MULTIPLIER),
                    -gamepad1.left_stick_x * (slowMode ? SLOW_MODE_MULTIPLIER : NORMAL_SPEED_MULTIPLIER),
                    -gamepad1.right_stick_x * (slowMode ? SLOW_MODE_MULTIPLIER : NORMAL_SPEED_MULTIPLIER),
                    ROBOT_CENTRIC
            );

            // Gamepad 1 Left Bumper: Toggle slow mode
            if (gamepad1.leftBumperWasPressed()) {
                slowMode = !slowMode;
            }
        }
    }}
