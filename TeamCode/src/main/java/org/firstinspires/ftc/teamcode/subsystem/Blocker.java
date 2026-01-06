package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Constants.BlockerConstants.*;
import org.firstinspires.ftc.teamcode.Robot;

public class Blocker {
    private ElapsedTime tapperFixTimer = new ElapsedTime(); // Timer for fixing tapper
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
        state = BlockerState.RAISED;
    }

    /**
     * Command the blocker to block (lower)
     */
    public void block() {
        state = BlockerState.BLOCKING;
    }

    /**
     * Command the blocker to fix a tapper jam
     */
    public void fixTapper() {
        state = BlockerState.FIX_TAPPER;
        tapperFixTimer.reset();
    }

    public void stop() {
        state = BlockerState.BLOCKING;
    }

    public void update(Robot robot) {
        /*
        * States:
        * BLOCKING - Stopping artifacts from bouncing up into the launcher
        * RAISED - Not blocking artifacts from entering the launcher
        * FIX_TAPPER - Moving to a position to fix a stuck artifact
        */
        switch (state) {
            case BLOCKING: // Default position
                robot.whackerServo.setPosition(BLOCK_POSITION);
                break;
            case RAISED:
                robot.whackerServo.setPosition(RAISED_POSITION);
                break;
            case FIX_TAPPER:
                robot.whackerServo.setPosition(FIX_TAPPER_POSITION);
                if (tapperFixTimer.milliseconds() >= TIME_TO_FIX_TAPPER) {
                    state = BlockerState.BLOCKING; // Return to blocking after fix duration
                }
                break;
        }
    }
}