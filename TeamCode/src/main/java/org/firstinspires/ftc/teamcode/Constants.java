package org.firstinspires.ftc.teamcode;

import com.pedropathing.control.FilteredPIDFCoefficients;

public class Constants {
    // Tapper
    public static class TapperConstants {
        public static double POSITIONING_TIME = 500; // Time it takes for the tapper to move between positions (milliseconds)
        public static double PUSHED_POSITION = 0.65; // Position that the tapper goes to when pushing an artifact into the launcher
        public static double HOME_POSITION = 0.1; // Position of the tapper when retracted
        public enum TapperState {
            IDLE,
            COMMANDED,
            PUSHED
        }
    }

    // Launcher
    public static class LauncherConstants {
        public static FilteredPIDFCoefficients leftLauncherCoefficients = new FilteredPIDFCoefficients(0, 0, 0, 0, 0);
        public static FilteredPIDFCoefficients rightLauncherCoefficients = new FilteredPIDFCoefficients(0, 0, 0, 0, 0);
        public static int TARGET_RPM = 710; // Target RPM for both launcher motors
        public static int RPM_TOLERANCE = 30; // Launch RPM tolerance (must be within the range of target RPM +- tolerance)
        public static int RPM_IN_RANGE_TIME = 300; // Time that the RPM must be within the tolerance before launching (milliseconds)
        public static int MIN_TIME_BETWEEN_LAUNCHES = 600; // Minimum time between launches (milliseconds)
        public enum LauncherState {
            IDLE,
            SPEED_UP,
            LAUNCH
        }

        public static class LauncherReturnProps { // Props returned by Launcher.update()
            public final LauncherState state;
            public final boolean cycleCompleted;

            public LauncherReturnProps(LauncherState state, boolean cycleCompleted) {
                this.state = state;
                this.cycleCompleted = cycleCompleted;
            }
        }
    }

    public static class IntakeConstants {
        public static double INTAKE_POWER = 0.8;
    }
}