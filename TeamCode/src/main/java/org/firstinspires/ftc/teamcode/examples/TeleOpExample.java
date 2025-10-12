package org.firstinspires.ftc.teamcode.examples;

// FTC SDK
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@TeleOp(name = "Example TeleOp", group = "Examples")
@Disabled // REMOVE THI LINE TO SEE ON DRIVER HUB
@SuppressWarnings("FieldCanBeLocal") // Stop Android Studio from bugging about variables being predefined
public class TeleOpExample extends LinearOpMode {
    private double slowModeMultiplier = 0.5; // Multiplier for slow mode speed
    private final double nonSlowModeMultiplier = 1; // Multiplier for normal driving speed
    private final boolean brakeMode = true; // Whether the motors should break on stop (recommended)
    private final boolean robotCentric = true; // True for robot centric driving, false for field centric
    public static Pose startingPose = new Pose(); // Starting pose of the robot for TeleOp

    private final ElapsedTime runtime = new ElapsedTime();
    private Follower follower; // Pedro pathing follower
    private Pose currentPose; // Current pose of the robot
    private boolean automatedDrive; // Is Pedro Pathing driving?
    private boolean slowMode = false; // Slow down the robot

    // Create path which moves to the line in front of the red goal from the current position
    // Use the Pedro Pathing Visualizer to see what this will do
    private final Supplier<PathChain> pathChain = () -> follower.pathBuilder()
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
        // Initialize the follower with the starting position, if it is null, assume 0, 0, 0
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();

        // Log completed initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the TeleOp period to start (driver presses START)
        waitForStart();
        runtime.reset();

        // Start with TeleOp (manual) drive
        follower.startTeleopDrive(brakeMode);

        while (opModeIsActive()) {
            // Update Pedro Pathing and Panels every iteration
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

            // Use A to follow the path
            if (gamepad1.aWasPressed()) {
                follower.followPath(pathChain.get()); // Follow path
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

            // X: Higher slow mode speed
            if (gamepad1.dpadUpWasReleased()) {
                slowModeMultiplier += 0.25;
            }

            // D Pad Down: Lower slow mode speed
            if (gamepad1.dpadDownWasReleased()) {
                slowModeMultiplier -= 0.25;
            }

            // Left Trigger: intake artifacts
            if (gamepad1.leftBumperWasReleased()) {
                intakeArtifacts();
            }

            // Right Trigger: shoot artifacts
            if (gamepad1.rightBumperWasReleased()) {
                shootArtifacts();
            }

            // Log status
            telemetry.addData("X: ", currentPose.getX());
            telemetry.addData("Y: ", currentPose.getY());
            telemetry.addData("Heading: ", currentPose.getHeading());
            telemetry.update();
        }
    }
}