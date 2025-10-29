package org.firstinspires.ftc.teamcode.autonomous;

// FTC SDK
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

// Local helper files
import org.firstinspires.ftc.teamcode.utils.Launcher;
import org.firstinspires.ftc.teamcode.utils.Pinpoint;
import org.firstinspires.ftc.teamcode.utils.Constants;

@Autonomous(name="LM1 Backup Autonomous", group="Autonomous")
public class LM1BackupAuto extends LinearOpMode {
    // Timer to track how long the robot has been moving
    private final ElapsedTime robotMovingFor = new ElapsedTime();
    private Pinpoint pinpoint;
    private Launcher launcher;
    private boolean launching = false;
    private boolean strafing = false;

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
        robotMovingFor.reset(); // Reset the robot move timer to zero

        while (opModeIsActive()) {
            pinpoint.update();

            if (!launching) {
                // Move diagonally forward and to the left
                double turn = 0;
                if (Math.toDegrees(pinpoint.getPosition().getHeading()) > 1) {
                    turn = 0.12;
                } else if (Math.toDegrees(pinpoint.getPosition().getHeading()) < 1) {
                    turn = -0.12;
                }

                double forward = -0.3;
                double strafe = 0.03;

                // Apply movement vectors to motors
                double frontLeftPower = forward + strafe + turn;
                double frontRightPower = forward - strafe - turn;
                double backLeftPower = forward - strafe + turn;
                double backRightPower = forward + strafe - turn;

                // Normalize the values so no wheel power exceeds 100%
                // This ensures that the robot maintains the desired motion.
                double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
                max = Math.max(max, Math.abs(backLeftPower));
                max = Math.max(max, Math.abs(backRightPower));
                if (max > 1.0) {
                    frontLeftPower /= max;
                    frontRightPower /= max;
                    backLeftPower /= max;
                    backRightPower /= max;
                }

                frontLeftDrive.setPower(frontLeftPower);
                frontRightDrive.setPower(frontRightPower);
                backLeftDrive.setPower(backLeftPower);
                backRightDrive.setPower(backRightPower);

                // If the robot has been moving for 1.5 seconds, stop the robot
                if (robotMovingFor.seconds() >= 2.8) {
                    frontLeftDrive.setPower(0);
                    frontRightDrive.setPower(0);
                    backLeftDrive.setPower(0);
                    backRightDrive.setPower(0);
                    launcher.speedUp();
                    launcher.launch(3);
                    launching = true;
                }
                launcher.update();
            } else {
                if (!strafing) {
                    Constants.LauncherConstants.LauncherReturnProps launcherReturn = launcher.update();
                    if (launcherReturn.cycleCompleted) {
                        strafing = true;
                        robotMovingFor.reset();
                    }
                } else {
                    frontLeftDrive.setPower(0.5);
                    frontRightDrive.setPower(-0.5);
                    backLeftDrive.setPower(-0.5);
                    backRightDrive.setPower(0.5);
                    if (robotMovingFor.seconds() > 1) {
                        frontLeftDrive.setPower(0);
                        frontRightDrive.setPower(0);
                        backLeftDrive.setPower(0);
                        backRightDrive.setPower(0);
                        break;
                    }
                }
            }
        }
    }
}
