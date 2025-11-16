package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Constants.LauncherConstants;

public class Robot {
    // Hardware
    private DcMotorEx leftLauncherMotor;
    private DcMotorEx rightLauncherMotor;
    private DcMotor intakeMotor;
    private Servo tapperServo;
    private Servo rampServo;

    public Robot(HardwareMap hardwareMap) {
        // NOTE: Drivetrain motors and Pinpoint are initialized and set up by Pedro Pathing
        leftLauncherMotor = hardwareMap.get(DcMotorEx.class, "launcher_left");
        rightLauncherMotor = hardwareMap.get(DcMotorEx.class, "launcher_right");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        tapperServo = hardwareMap.get(Servo.class, "tapper");
        rampServo = hardwareMap.get(Servo.class, "ramp");

        // Launcher motor configuration
        PIDFCoefficients launcherPIDF = new PIDFCoefficients(LauncherConstants.P, LauncherConstants.I, LauncherConstants.D, LauncherConstants.F);
        leftLauncherMotor.setZeroPowerBehavior(BRAKE);
        leftLauncherMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftLauncherMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, launcherPIDF);
        rightLauncherMotor.setDirection(REVERSE);
        rightLauncherMotor.setZeroPowerBehavior(BRAKE);
        rightLauncherMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightLauncherMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, launcherPIDF);

        // Intake motor configuration
        intakeMotor.setZeroPowerBehavior(BRAKE);
        intakeMotor.setDirection(REVERSE);

        // TODO: Initialize subsystems
        /*
        Game plan for codebase rewrite:
        - Subsystems:
            - Each subsystem will have extend a base Subsystem class
                - It has an update() method that is called in Robot.update(), the method takes a telemetry object as a parameter for logging
                - It has a stop() method that stops all motors/servos in the subsystem
                - Each subsystem can override these methods as needed (by default they do nothing)
            - The Robot.update() method will be called in the main OpMode loop to update all subsystems
            - Robot.stop() method will call stop() on all subsystems to ensure everything is stopped safely
                - The OpMode can also call Robot.subsystem.stop() directly if needed
            - Launcher, Intake, Camera, Gamepad, and Drivetrain will be subsystems
        - Helpers:
            - Helper methods that previously resided in their own class will be moved in here
                - AllianceSelector will become Robot.selectAlliance()
                - StartPositionSelector will become Robot.selectStartPosition()
        - Autonomous:
            - Autonomous OpModes will use a state machine loop
                - Each state will be represented by an enum value
                - There will be a list of states at the top of the OpMode for easy configuration
                - The state machine will use the Robot method to perform actions during Autonomous
        - TeleOp:
            - TeleOp OpModes will use the Robot methods to control the robot
                - Gamepad inputs will be processed in the OpMode and passed to the Robot methods as needed
                - Use the Gamepad subsystem to handle gamepad input processing and lighting
        - Pedro Pathing:
            - Pedro Pathing integration will be handled in the Drivetrain subsystem
                - The Drivetrain subsystem will have methods to initialize and use Pedro Pathing
            - Separate Constants file from the main one to prevent problems with PIDFCoefficients import and Tuning file problems
         */
    }

    public void update() {
        // TODO: Update subsystems
    }
}
