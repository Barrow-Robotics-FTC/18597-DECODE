package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Constants.BlockerConstants.*;
import org.firstinspires.ftc.teamcode.Robot;

public class Blocker {
    private final ElapsedTime moveTimer = new ElapsedTime();
    private BlockerState state = BlockerState.BLOCKING; // Current state of the tapper

    // Constructor
    public Blocker(Robot robot) {
        update(robot); // Run an initial update to set the servo position
    }

    /**
     * Get the current state of the blocker
     *
     * @return The current BlockerState
     */
    public BlockerState getState() {
        return state;
    }

    /**
     * Check if the blocker is raised
     *
     * @return True if the whacker is raised, false otherwise
     */
    public boolean isFinishedRaising() {
        return state == BlockerState.RAISED;
    }

    /**
     * Check if the blocker is blocking
     *
     * @return True if the blocker is blocking, false otherwise
     */
    public boolean isBlocking() {
        return state == BlockerState.BLOCKING;
    }

    /**
     * Command the blocker to raise
     */
    public void raise() {
        if (state == BlockerState.RAISED || state == BlockerState.RAISING) {
            return; // Already raised or in the process of raising
        }
        state = BlockerState.RAISING;
        moveTimer.reset(); // Start timer to track positioning time
    }

    /**
     * Command the blocker to block (lower)
     */
    public void block() {
        state = BlockerState.BLOCKING;
    }

    public void stop() {
        state = BlockerState.BLOCKING;
    }

    public void update(Robot robot) {
        /*
        * States:
        * BLOCKING - Stopping artifacts from bouncing up into the launcher
        * RAISING - Moving from blocking position to raised position
        * RAISED - Not blocking artifacts from entering the launcher
        */
        switch (state) {
            case BLOCKING: // Default position
                robot.whackerServo.setPosition(BLOCK_POSITION);
                break;
            case RAISING:
                robot.whackerServo.setPosition(RAISED_POSITION);
                if (moveTimer.seconds() >= POSITIONING_TIME) {
                    state = BlockerState.RAISED; // Transition to raised state after time elapsed
                }
            case RAISED:
                robot.whackerServo.setPosition(RAISED_POSITION);
                break;
        }
    }
}