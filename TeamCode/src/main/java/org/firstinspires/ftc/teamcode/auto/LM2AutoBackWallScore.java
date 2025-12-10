package org.firstinspires.ftc.teamcode.auto;

// FTC SDK
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

// Local helper files
import static org.firstinspires.ftc.teamcode.helper.Constants.AprilTagConstants.BLUE_GOAL_TAG_ID;
import static org.firstinspires.ftc.teamcode.helper.Constants.AprilTagConstants.RED_GOAL_TAG_ID;
import org.firstinspires.ftc.teamcode.helper.AllianceSelector;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.Launcher;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Camera;
import org.firstinspires.ftc.teamcode.helper.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name="LM2 Autonomous (Back Wall Score)", group="Autonomous")
@SuppressWarnings("FieldCanBeLocal") // Suppress pointless Android Studio warnings
public class LM2AutoBackWallScore extends LinearOpMode {
    // Seconds to wait for April tag detection before timing out
    private final int APRIL_TAG_NULL_TIMEOUT = 5000; // Milliseconds

    // Set a static time to wait before starting auto
    private final int TIME_BEFORE_STARTING = 0; // Seconds
    private final ElapsedTime runtime = new ElapsedTime();

    // Utilities
    private final ElapsedTime nullAprilTagTimeoutTimer = new ElapsedTime();
    private Drivetrain drivetrain;
    private Launcher launcher;
    private Intake intake;
    private Camera camera;

    // Variables
    enum State {
        START,
        MOVING_FORWARD,
        TURNING,
        LOCALIZING,
        LAUNCHING,
        STRAFING,
        COMPLETE
    }
    private State currentState = State.START; // Current state of the robot
    private boolean exitAuto = false; // Should the auto be stopped
    private Constants.Alliance alliance; // Alliance of the robot
    private ElapsedTime robotMovingFor;

    @Override
    public void runOpMode() {
        // Initialize subsystems
        drivetrain = new Drivetrain(hardwareMap, Drivetrain.Mode.TELEOP);
        launcher = new Launcher(hardwareMap);
        intake = new Intake(hardwareMap);
        camera = new Camera(hardwareMap);
        //camera.stop(); // Stop camera until needed

        // Prompt the driver to select an alliance
        alliance = AllianceSelector.run(gamepad1, telemetry);
        blackboard.put("alliance", alliance); // Save the alliance for TeleOp

        // Tell the driver that initialization is complete
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset(); // Reset the runtime timer to zero

        while (opModeIsActive()) {
            drivetrain.update(); // Update drivetrain
            launcher.update(); // Update the launcher
            intake.update(launcher);

            if (exitAuto) {
                break; // Exit the auto if the exit flag is set
            }

            switch (currentState) {
                case START:
                    // Wait for the specified time before starting auto
                    if (runtime.seconds() >= TIME_BEFORE_STARTING) {
                        currentState = State.MOVING_FORWARD ; // Start lining up
                    }

                    break;
                case MOVING_FORWARD:
                    // Move backwards until a set distance from the AprilTag is reached
                    if (drivetrain.getPose().getX() < 56) {
                        // Use the Pinpoint to correct for heading error while moving back
                        double forward = 0.4; // Move back at 0.3 power
                        double strafe = 0; // No strafing
                        double turn = 0;
                        double currHeading = Math.toDegrees(drivetrain.getPose().getHeading());
                        if (Math.abs(currHeading) > 1) {
                            turn = currHeading > 0 ? -0.1 : 0.1; // Small turn to correct heading
                        }

                        // Set drivetrain movement vectors
                        drivetrain.setMovementVectors(new Constants.MovementVectors(forward, strafe, turn));
                    } else {
                        // Stop the robot
                        drivetrain.stop();

                        // Move to the localizing state
                        currentState = State.TURNING;
                    }

                    break;
                case TURNING:
                    // Move backwards until a set distance from the AprilTag is reached
                    if (Math.abs(Math.toDegrees(drivetrain.getPose().getHeading())) < 45) {
                        // Use the Pinpoint to correct for heading error while moving back
                        double forward = 0; // no forward
                        double strafe = 0; // No strafing
                        double turn = alliance == Constants.Alliance.BLUE ? 0.25 : -0.25;

                        // Set drivetrain movement vectors
                        drivetrain.setMovementVectors(new Constants.MovementVectors(forward, strafe, turn));
                    } else {
                        // Stop the robot
                        drivetrain.stop();

                        // Move to the localizing state
                        currentState = State.LOCALIZING;
                    }
                    break;
                case LOCALIZING:
                    // Line up with the April Tag
                    boolean moveOn = false;
                    AprilTagDetection tag = camera.getAprilTag(alliance == Constants.Alliance.BLUE ? BLUE_GOAL_TAG_ID : RED_GOAL_TAG_ID);
                    if (tag != null) {
                        Constants.MovementVectors alignmentVectors = camera.driveToAprilTag(tag, Constants.DISTANCE_FROM_APRIL_TAG);
                        if (alignmentVectors.moveCompleted) {
                            moveOn = true; // Alignment complete, move on
                        } else {
                            drivetrain.setMovementVectors(alignmentVectors); // Set drivetrain movement vectors
                        }
                        nullAprilTagTimeoutTimer.reset(); // Reset timeout timer
                    } else {
                        // No April tag detected
                        if (nullAprilTagTimeoutTimer.milliseconds() > APRIL_TAG_NULL_TIMEOUT) {
                            // Timeout reached, stop trying to detect April Tag
                            moveOn = true;
                        }
                    }

                    // If alignment is complete, stop and move to launching
                    if (moveOn) {
                        drivetrain.stop(); // Stop drivetrain movement
                        //camera.stop(); // Camera isn't needed anymore

                        // Start launching
                        launcher.speedUp();
                        launcher.launch(3);
                        currentState = State.LAUNCHING; // Move to the launching state
                    }

                    break;
                case LAUNCHING:
                    // Wait for the launcher to finish launching
                    Constants.LauncherConstants.LauncherReturnProps launcherReturn = launcher.update();
                    if (launcherReturn.cycleCompleted) {
                        robotMovingFor.reset();
                        currentState = State.STRAFING;
                    }

                    break;
                case STRAFING:
                    if (robotMovingFor.milliseconds() < 1500) {
                        drivetrain.setMovementVectors(new Constants.MovementVectors(0, -0.5, 0)); // Strafe right at 0.5 power
                    } else {
                        drivetrain.stop(); // Stop the robot
                        currentState = State.COMPLETE; // Move to the complete state
                    }

                    break;
                case COMPLETE:
                    launcher.stop(); // Stop the launcher
                    exitAuto = true; // Set exit flag to true

                    break;
            }

            telemetry.addData("State", currentState);
            telemetry.update();
        }
    }
}