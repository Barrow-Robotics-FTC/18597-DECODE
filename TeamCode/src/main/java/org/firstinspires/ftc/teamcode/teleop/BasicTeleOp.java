package org.firstinspires.ftc.teamcode.teleop;

// FTC SDK
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

// Panels
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

// Pedro Pathing
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

// Java
import java.util.function.Supplier;

@TeleOp(name = "Basic TeleOp", group = "TeleOp")
@Configurable // Use Panels
@SuppressWarnings("FieldCanBeLocal") // Stop Android Studio from bugging about variables being predefined
public class BasicTeleOp extends LinearOpMode {
    private double slowModeMultiplier = 0.5; // Multiplier for slow mode speed
    private final double nonSlowModeMultiplier = 1; // Multiplier for normal driving speed
    private final boolean brakeMode = true; // Whether the motors should break on stop (recommended)
    private final boolean robotCentric = true; // True for robot centric driving, false for field centric
    private final ElapsedTime runtime = new ElapsedTime();
    private Follower follower; // Pedro pathing follower
    private Pose currentPose; // Current pose of the robot
    private boolean automatedDrive; // Is Pedro Pathing driving?
    private TelemetryManager panelsTelemetry; // Panels telemetry
    private boolean slowMode = false; // Slow down the robot

    // Create path which moves to the line in front of the red goal from the current position
    // Use the Pedro Pathing Visualizer to see what this will do
    private final Supplier<PathChain> scorePosePath = () -> follower.pathBuilder()
            .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
            .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(135), 0.8))
            .build();

    private void intakeArtifacts() {
        // Put your intake logic here
        return;
    }

    private void shootArtifacts() {
        // Put your shooting logic here
        return;
    }

    @Override
    public void runOpMode() {
        // Initialize the Pedro Pathing follower and set the start pose to the autonomous ending pose
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());
        follower.update();

        // Initialize Panels telemetry
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Log completed initialization to Panels and driver station
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry); // Update Panels and driver station after logging

        // Wait for the TeleOp period to start (driver presses START)
        waitForStart();
        runtime.reset();

        // Start with TeleOp (manual) drive
        follower.startTeleopDrive(brakeMode);

        while (opModeIsActive()) {
            // Update Pedro Pathing every iteration
            follower.update();
            currentPose = follower.getPose();

            if (!automatedDrive) {
                // Send gamepad inputs to Pedro Pathing for driving
                // Make the last parameter false for field-centric
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * (slowMode ? slowModeMultiplier : nonSlowModeMultiplier),
                        -gamepad1.left_stick_x * (slowMode ? slowModeMultiplier : nonSlowModeMultiplier),
                        -gamepad1.right_stick_x * (slowMode ? slowModeMultiplier : nonSlowModeMultiplier),
                        robotCentric
                );
            }

            // Use A to follow the path to score
            if (gamepad1.aWasPressed()) {
                follower.followPath(scorePosePath.get()); // Follow path
                automatedDrive = true;
            }

            // Stop automated following if the follower is done or the driver presses B
            if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
                follower.startTeleopDrive(brakeMode); // Restart the manual TeleOp drive
                automatedDrive = false;
            }

            // Right bumper enables slow mode
            if (gamepad1.yWasReleased()) {
                slowMode = !slowMode;
            }

            // Left bumper disables slow mode
            if (gamepad1.yWasReleased()) {
                slowMode = !slowMode;
            }

            // X: Higher slow mode speed
            if (gamepad1.dpadUpWasReleased()) {
                slowModeMultiplier += 0.25;
            }

            // D Pad Down: Lower slow mode speed
            if (gamepad1.dpadDownWasReleased()) {
                slowModeMultiplier -= 0.25;
            }

            // Left Trigger: intake artifacts
            if (gamepad2.leftBumperWasReleased()) {
                intakeArtifacts();
            }

            // Right Trigger: shoot artifacts
            if (gamepad2.rightBumperWasReleased()) {
                shootArtifacts();
            }

            // Log status to Panels and driver station
            panelsTelemetry.debug("X: ", currentPose.getX());
            panelsTelemetry.debug("Y: ", currentPose.getY());
            panelsTelemetry.debug("Heading: ", currentPose.getHeading());
            panelsTelemetry.update(telemetry); // Update Panels and Driver Station after logging
        }
    }
}