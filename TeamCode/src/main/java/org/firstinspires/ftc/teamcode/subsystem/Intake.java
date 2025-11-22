package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.Constants.IntakeConstants.*;
import org.firstinspires.ftc.teamcode.Robot;

public class Intake {
    // Other variables
    private IntakeState state = IntakeState.STOPPED; // Current state of the intake

    // Constructor
    public Intake(Robot robot) {
        update(robot); // Run an initial update (STOPPED) to set the intake ramp to the home position
    }

    /**
     * Get the current state of the tapper
     *
     * @return The current IntakeState
     */
    public IntakeState getState() {
        return state;
    }

    /**
     * Check if the intake is running
     *
     * @return True if the intake is running, false otherwise
     */
    public boolean isActive() {
        return state == IntakeState.RUNNING;
    }

    /**
     * Command the intake motor to run and lower the ramp to intake artifacts
     */
    public void run() {
        state = IntakeState.RUNNING;
    }

    /**
     * Command the intake motor to stop and the ramp to hold artifacts in the storage area
     */
    public void stop() {
        state = IntakeState.STOPPED;
    }

    public void update(Robot robot) {
        /*
         * States:
         * STOPPED - Intake motor is off, ramp is holding artifacts in the storage area
         * RUNNING - Intake motor is on, ramp is lowered to intake artifacts
         */
        switch (state) {
            case RUNNING:
                robot.intakeMotor.setPower(INTAKE_POWER); // Run intake motor at set power
                robot.rampServo.setPosition(RAMP_INTAKE_POSITION); // Lower the intake ramp
                break;
            case STOPPED:
                robot.intakeMotor.setPower(0); // Stop the intake motor
                robot.rampServo.setPosition(RAMP_HOLD_POSITION); // Hold artifacts in the storage area
                break;
        }
    }
}