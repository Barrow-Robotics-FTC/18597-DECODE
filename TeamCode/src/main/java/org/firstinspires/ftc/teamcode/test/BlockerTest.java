package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot;

/*
Blocker Test OpMode

*/

@TeleOp(name = "Blocker Test", group = "Tests")
@SuppressWarnings("FieldCanBeLocal") // Suppress pointless Android Studio warnings
public class BlockerTest extends LinearOpMode {
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
                if (robot.blocker.isBlocking()) {
                    robot.blocker.raise();
                } else {
                    robot.blocker.block();
                }
            }

            // Log status
            telemetry.addData("Whacker State", robot.blocker.getState());
            telemetry.update();
        }
    }
}