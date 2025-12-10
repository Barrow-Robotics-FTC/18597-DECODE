package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Constants.WhackerConstants.*;
import org.firstinspires.ftc.teamcode.Robot;

public class Whacker {
    // Timers
    private final ElapsedTime whackTimer = new ElapsedTime();

    // Other variables
    private WhackerState state = WhackerState.IDLE; // Current state of the tapper

    // Constructor
    public Whacker(Robot robot) {
        update(robot); // Run an initial update (IDLE) to set the whacker to the home position
    }

    /**
     * Get the current state of the whacker
     *
     * @return The current WhackerState
     */
    public WhackerState getState() {
        return state;
    }

    /**
     * Check if the whacker is whacking
     *
     * @return True if the whacker is whacking, false otherwise
     */
    public boolean isWhacking() {
        return state == WhackerState.WHACKING;
    }

    /**
     * Command the whacker to whack the artifacts
     */
    public void whack() {
        if (state == WhackerState.WHACKING) {
            return; // Already whacking
        }
        state = WhackerState.WHACKING;
        whackTimer.reset(); // Start timer to track positioning time
    }

    public void stop() {
        if (state == WhackerState.IDLE) {
            return; // Already idle
        }
        state = WhackerState.IDLE;
    }

    public void update(Robot robot) {
        /*
        * States:
        * IDLE - Whacker is in the home position
        * WHACKING - Whacker is in the process of whacking artifacts
        * RESETTING - Whacker is returning to the home position after whacking
        */
        switch (state) {
            case IDLE:
                // Maintain home position
                robot.whackerServo.setPosition(HOME_POSITION);
                break;
            case WHACKING:
                // Move to whack position
                robot.tapperServo.setPosition(WHACK_POSITION);

                // Use the timer to determine when the tapper has reached the position
                if (whackTimer.milliseconds() >= POSITIONING_TIME) {
                    state = WhackerState.RESETTING;
                    whackTimer.reset(); // Used for resetting too
                }
                break;
             case RESETTING:
                // Move to home position
                robot.whackerServo.setPosition(HOME_POSITION);

                // Use the timer to determine when the whacker has reached the home position
                if (whackTimer.milliseconds() >= POSITIONING_TIME) {
                    state = WhackerState.IDLE; // Finished resetting
                }

                break;
        }
    }
}