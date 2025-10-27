package org.firstinspires.ftc.teamcode.autonomous;

// FTC SDK
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

// Local helper files
import org.firstinspires.ftc.teamcode.utils.Constants;

@Autonomous(name="LM1 Backup To the Backup Autonomous", group="Autonomous")
public class LM1BackupToTheBackupAuto extends LinearOpMode {
    // Timer to track how long the robot has been moving
    private final ElapsedTime robotMovingFor = new ElapsedTime();

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

        // Tell the driver that initialization is complete
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        robotMovingFor.reset(); // Reset the robot move timer to zero

        while (opModeIsActive()) {
            // Move diagonally forward and to the left
            frontLeftDrive.setPower(-.5);
            frontRightDrive.setPower(.5);
            backLeftDrive.setPower(.5);
            backRightDrive.setPower(-.5);

            // If the robot has been moving for 1.5 seconds, stop the robot
            if (robotMovingFor.seconds() >= 1.5) {
                frontLeftDrive.setPower(0);
                frontRightDrive.setPower(0);
                backLeftDrive.setPower(0);
                backRightDrive.setPower(0);
                break; // Exit the loop (end OpMode)
            }
        }
    }
}
