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
        - Launcher:
            - We'll switch from 6000RPM motors to 1620RPM motors for better control and efficiency
                - Its easier to run a motor at 50% power than 100% power for velocity control
                - Faster speed ups due to more torque at lower speeds
                - We don't use the full 6000RPM, so there's no need for it
                - Saves energy due to these things
            - Use the Pedro Pathing Filtered PIDF Controller to manage launcher velocity
            - Tuning
                - F: Provides the "base" amount of power needed to hit the target before any error even occurs.
                    - Set everything to 0
                    - Increase F until the motor settles near the target velocity (about 90%)
                - T: Controls how much to smooth the derivative signal. It's a value between 0 (no filtering) and 1 (max filtering, but very laggy)
                    - Start with T = 0.7 (70% old value, 30% new value)
                    - Tune with D, if the motor is "chattering", increase T, if it's too slow to respond, decrease T
                - D: Fights against rapid changes in error, which is what happens before overshoot and oscillation.
                    - Start with D = 0
                    - Set P to very small value
                    - Increase D until the motor stops overshooting and oscillating
                    - Simultaneously tune T
                - P: Provides power that is proportional to the current error. It does the main work of closing the gap from where F left off
                    - Slowly increase P until the motor gets to the target quickly without overshooting
                    - D can be further adjusted to reduce any overshoot
                - I: Fixes any steady-state error that may occur when the motor is at the target velocity
                    - Keep I = 0 unless you notice any steady-state error (ex. motor settles below target)
                    - If needed, slowly increase I until the motor reaches the target exactly
            - Make the launcher subsystem first and implement this into it, make a LauncherTuner and LauncherTest OpMode to help with tuning and testing
                - Don't worry about integrating with Robot class for now, just make it work standalone
         */
    }

    public void update() {
        // TODO: Update subsystems
    }
}
