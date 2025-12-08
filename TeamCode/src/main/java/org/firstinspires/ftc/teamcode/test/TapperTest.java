package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot;

/*
Position Recorder OpMode
Displays the current position of the robot in telemetry to help record positions for autonomous paths
Start the robot in the blue alliance human player zone corner (localize position)
*/

@TeleOp(name = "Tapper Test", group = "Tests")
@SuppressWarnings("FieldCanBeLocal") // Suppress pointless Android Studio warnings
public class TapperTest extends LinearOpMode {
    private Robot robot; // Custom robot class

    @Override
    public void runOpMode() {
        // Create instance of launcher and initialize
        robot = new Robot(hardwareMap, Constants.Mode.TELEOP);

        // Log completed initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for START to be pressed
        waitForStart();

        while (opModeIsActive()) {
            // Update robot
            robot.update(gamepad1, gamepad2);

            if (gamepad1.rightBumperWasPressed()) {
                if (robot.tapper.isInIdlePosition()) {
                    robot.tapper.push();
                } else {
                    robot.tapper.retract();
                }
            }

            // Log status
            telemetry.addData("Tapper State", robot.tapper.getState());
            telemetry.update();
        }
    }
}