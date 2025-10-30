package org.firstinspires.ftc.teamcode.autonomous;

// FTC SDK
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

// Pedro Pathing (follower is not actually used)
import com.pedropathing.geometry.Pose;

// Local helper files
import org.firstinspires.ftc.teamcode.utils.Launcher;
import org.firstinspires.ftc.teamcode.utils.Pinpoint;
import org.firstinspires.ftc.teamcode.utils.Constants;

@Autonomous(name="LM1 Autonomous", group="Autonomous", preselectTeleOp="LM1TeleOp")
@SuppressWarnings("FieldCanBeLocal") // Suppress pointless Android Studio warnings
public class LM1Auto extends LinearOpMode {
    // Set a static time to wait before starting auto
    private final int TIME_BEFORE_STARTING = 0; // Seconds
    private final ElapsedTime runtime = new ElapsedTime();

    // Utilities
    private final ElapsedTime robotMovingFor = new ElapsedTime();
    private Pinpoint pinpoint;
    private Launcher launcher;

    // Variables
    enum State {
        START,
        LINING_UP,
        LAUNCHING,
        STRAFING,
        COMPLETE
    }
    private State currentState = State.START; // Current state of the robot
    private boolean exitAuto = false; // Should the auto be stopped

    @Override
    public void runOpMode() {
        // Initialize hardware
        DcMotor frontLeftDrive = hardwareMap.get(DcMotor.class, Constants.Pedro.driveConstants.leftFrontMotorName);
        DcMotor frontRightDrive = hardwareMap.get(DcMotor.class, Constants.Pedro.driveConstants.rightFrontMotorName);
        DcMotor backLeftDrive = hardwareMap.get(DcMotor.class, Constants.Pedro.driveConstants.leftRearMotorName);
        DcMotor backRightDrive = hardwareMap.get(DcMotor.class, Constants.Pedro.driveConstants.rightRearMotorName);

        // Set motor directions
        frontLeftDrive.setDirection(Constants.Pedro.driveConstants.leftFrontMotorDirection);
        frontRightDrive.setDirection(Constants.Pedro.driveConstants.rightFrontMotorDirection);
        backLeftDrive.setDirection(Constants.Pedro.driveConstants.leftRearMotorDirection);
        backRightDrive.setDirection(Constants.Pedro.driveConstants.rightRearMotorDirection);

        // Set motors to brake mode
        frontLeftDrive.setZeroPowerBehavior(BRAKE);
        frontRightDrive.setZeroPowerBehavior(BRAKE);
        backLeftDrive.setZeroPowerBehavior(BRAKE);
        backRightDrive.setZeroPowerBehavior(BRAKE);

        pinpoint = new Pinpoint(hardwareMap);
        pinpoint.setPosition(new Pose(0, 0, 0));
        launcher = new Launcher(hardwareMap);

        // Tell the driver that initialization is complete
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset(); // Reset the runtime timer to zero
        robotMovingFor.reset(); // Reset the robot move timer to zero

        while (opModeIsActive()) {
            pinpoint.update(); // Update current position
            launcher.update(); // Update the launcher

            if (exitAuto) {
                break; // Exit the auto if the exit flag is set
            }

            switch (currentState) {
                case START:
                    // Wait for the specified time before starting auto
                    if (runtime.seconds() >= TIME_BEFORE_STARTING) {
                        currentState = State.LINING_UP; // Start lining up
                    }
                    break;
                case LINING_UP:
                    // Use the Pinpoint to correct for heading error while moving back
                    double turn = 0;
                    if (Math.toDegrees(pinpoint.getPosition().getHeading()) > 1) {
                        turn = 0.12;
                    } else if (Math.toDegrees(pinpoint.getPosition().getHeading()) < 1) {
                        turn = -0.12;
                    }

                    // Static movement values
                    double forward = -0.3; // Move back at 0.3 power
                    double strafe = 0.03; // Tiny correction to the right for

                    // Apply movement vectors to motors
                    double frontLeftPower = forward + strafe + turn;
                    double frontRightPower = forward - strafe - turn;
                    double backLeftPower = forward - strafe + turn;
                    double backRightPower = forward + strafe - turn;

                    // Normalize the values so no wheel power exceeds 100%
                    // This ensures that the robot maintains the desired motion.
                    double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
                    max = Math.max(max, Math.abs(backLeftPower));
                    //noinspection DataFlowIssue
                    max = Math.max(max, Math.abs(backRightPower));
                    //noinspection ReassignedVariable,ConstantValue
                    if (max > 1.0) {
                        frontLeftPower /= max;
                        frontRightPower /= max;
                        backLeftPower /= max;
                        backRightPower /= max;
                    }

                    // Set powers
                    frontLeftDrive.setPower(frontLeftPower);
                    frontRightDrive.setPower(frontRightPower);
                    backLeftDrive.setPower(backLeftPower);
                    backRightDrive.setPower(backRightPower);

                    // If the robot has been moving for 2.8 seconds, stop
                    if (robotMovingFor.seconds() >= 2.8) {
                        frontLeftDrive.setPower(0);
                        frontRightDrive.setPower(0);
                        backLeftDrive.setPower(0);
                        backRightDrive.setPower(0);

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
                        currentState = State.STRAFING;
                        robotMovingFor.reset();
                    }
                    break;
                case STRAFING:
                    // Strafe right off the launch line for a second
                    frontLeftDrive.setPower(0.5);
                    frontRightDrive.setPower(-0.5);
                    backLeftDrive.setPower(-0.5);
                    backRightDrive.setPower(0.5);
                    if (robotMovingFor.seconds() > 1) {
                        frontLeftDrive.setPower(0);
                        frontRightDrive.setPower(0);
                        backLeftDrive.setPower(0);
                        backRightDrive.setPower(0);
                        currentState = State.COMPLETE;
                    }
                    break;
                case COMPLETE:
                    launcher.stop(); // Stop the launcher
                    exitAuto = true; // Set exit flag to true
                    break;
            }
        }
    }
}
