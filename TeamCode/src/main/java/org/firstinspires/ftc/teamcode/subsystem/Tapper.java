package org.firstinspires.ftc.teamcode.subsystem;

// FTC SDK
import com.qualcomm.robotcore.util.ElapsedTime;

// Local helpers
import static org.firstinspires.ftc.teamcode.Constants.TapperConstants.*;
import org.firstinspires.ftc.teamcode.Robot;

public class Tapper {
    // Timers
    private final ElapsedTime tapperRaisedTimer = new ElapsedTime();

    // Other variables
    private TapperState state = TapperState.IDLE; // Current state of the tapper

    // Constructor
    public Tapper(Robot robot) {
        update(robot); // Run an initial update (IDLE) to set the tapper to the home position
    }

    /**
     * Get the current state of the tapper
     *
     * @return The current TapperState
     */
    public TapperState getState() {
        return state;
    }

    /**
     * Check if the tapper is in the pushed position
     *
     * @return True if the tapper is pushed, false otherwise
     */
    public boolean isPushed() {
        return state == TapperState.PUSHED;
    }

    /**
     * Command the tapper to push an artifact into the launcher
     */
    public void push() {
        if (state == TapperState.COMMANDED || state == TapperState.PUSHED) {
            return; // Already commanded or pushed
        }
        state = TapperState.COMMANDED;
        tapperRaisedTimer.reset(); // Start timer to track positioning time
    }

    /**
     * Stop the tapper and return it to the home position
     */
    public void retract() {
        state = TapperState.IDLE;
    }

    public void stop() {
        retract();
    }

    public void update(Robot robot) {
        /*
        * States:
        * IDLE: Hold home position
        * COMMANDED: Move to pushed position
        * PUSHED: Hold pushed position
        */
        switch (state) {
            case IDLE:
                // Hold home position
                robot.tapperServo.setPosition(HOME_POSITION);
                break;
            case COMMANDED:
                // Move to pushed position
                robot.tapperServo.setPosition(PUSHED_POSITION);

                // Use the timer to determine when the tapper has reached the position
                if (tapperRaisedTimer.milliseconds() >= POSITIONING_TIME) {
                    state = TapperState.PUSHED;
                }
                break;
            case PUSHED:
                // Hold pushed position
                robot.tapperServo.setPosition(PUSHED_POSITION);
                break;
        }
    }
}