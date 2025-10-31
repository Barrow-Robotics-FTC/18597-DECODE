package org.firstinspires.ftc.teamcode.autonomous;

// FTC SDK
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

// Local helper files
import org.firstinspires.ftc.teamcode.utils.AllianceSelector;
import org.firstinspires.ftc.teamcode.utils.Constants;

@Autonomous(name="LM1 Autonomous (Back Wall)", group="Autonomous", preselectTeleOp="LM1TeleOp")
@SuppressWarnings("FieldCanBeLocal") // Suppress pointless Android Studio warnings
public class LM1AutoBackWall extends LinearOpMode {
    // Set a static time to wait before starting auto
    private final int TIME_BEFORE_STARTING = 0; // Seconds
    private final ElapsedTime runtime = new ElapsedTime();

    // Utilities
    private final ElapsedTime robotMovingFor = new ElapsedTime();

    // Variables
    private Constants.Alliance alliance; // Alliance of the robot

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

        // Prompt the driver to select an alliance
        alliance = AllianceSelector.run(gamepad1, telemetry);
        blackboard.put("alliance", alliance); // Save the alliance for TeleOp

        // Tell the driver that initialization is complete
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset(); // Reset the runtime timer to zero
        robotMovingFor.reset(); // Reset the robot move timer to zero

        while (opModeIsActive()) {
            if (runtime.seconds() >= TIME_BEFORE_STARTING) { // Wait for the specified time before starting auto
                // Move forward at half power
                frontLeftDrive.setPower(0.5);
                frontRightDrive.setPower(0.5);
                backLeftDrive.setPower(0.5);
                backRightDrive.setPower(0.5);

                // If the robot has been moving for 1 second, stop the robot
                if (robotMovingFor.seconds() >= 1) {
                    frontLeftDrive.setPower(0);
                    frontRightDrive.setPower(0);
                    backLeftDrive.setPower(0);
                    backRightDrive.setPower(0);
                    break; // Exit the loop (end OpMode)
                }
            }
        }
    }
}
