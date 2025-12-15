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
        return state == WhackerState.PUSHED;
    }

    /**
     * Command the whacker to whack the artifacts
     */
    public void push() {
        state = WhackerState.PUSHED;
        whackTimer.reset(); // Start timer to track positioning time
    }

    public void stop() {
        state = WhackerState.IDLE;
    }

    public void update(Robot robot) {
        /*
        * States:
        * IDLE - Whacker is in the home position
        * PUSHED - Stopping artifacts from bouncing up
        */
        switch (state) {
            case IDLE:
                // Maintain home position
                robot.whackerServo.setPosition(HOME_POSITION);
                break;
            case PUSHED:
                robot.whackerServo.setPosition(WHACK_POSITION);
                break;
        }
    }
}