package org.firstinspires.ftc.teamcode.command;

import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Base class for command-based OpModes using Pedro Pathing Ivy
 */
public abstract class CommandOpMode extends OpMode {

    /**
     * Cancels all previous commands
     */
    public void reset() {
        Scheduler.getInstance().reset();
    }

    /**
     * Schedules commands to the scheduler
     */
    public void schedule(ICommand... commands) {
        Scheduler.getInstance().schedule(commands);
    }

    @Override
    public void init() {
    }

    @Override
    public void loop() {
        Scheduler.getInstance().execute();
    }

    @Override
    public void stop() {
        reset();
    }
}
