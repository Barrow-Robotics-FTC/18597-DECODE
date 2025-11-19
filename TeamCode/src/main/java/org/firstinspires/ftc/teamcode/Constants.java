package org.firstinspires.ftc.teamcode;

import com.pedropathing.control.FilteredPIDFCoefficients;

public class Constants {
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

    public static class LauncherConstants {
        public static FilteredPIDFCoefficients leftLauncherCoefficients = new FilteredPIDFCoefficients(0, 0, 0, 0, 0);
        public static FilteredPIDFCoefficients rightLauncherCoefficients = new FilteredPIDFCoefficients(0, 0, 0, 0, 0);
        public static int TARGET_RPM = 710; // Target RPM for both launcher motors
        public static int RPM_TOLERANCE = 30; // Launch RPM tolerance (must be within the range of target RPM +- tolerance)
        public static int RPM_IN_RANGE_TIME = 300; // Time that the RPM must be within the tolerance before launching (milliseconds)
        public static int MIN_TIME_BETWEEN_LAUNCHES = 600; // Minimum time between launches (milliseconds)
        public static double LAUNCHER_POWER_WHILE_INTAKING = 0.3; // Power for launcher wheels while intaking to prevent jamming
        public static double POSITIVE_TO_NEGATIVE_SAFETY_RPM = 100; // RPM threshold to prevent sudden direction changes when switching directions for intake
        public static double COAST_DOWN_POWER = -0.1; // Power to apply to the launcher wheels to help them coast down below the safety threshold when reversing for intake
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
        public static double INTAKE_POWER = 0.8; // Power for intake motor
        public static double RAMP_HOLD_POSITION = 1.0; // Position for the intake ramp that holds artifacts in the storage area
        public static double RAMP_INTAKE_POSITION = 0.0; // Position for the intake ramp that allows the intake to put artifacts in the storage area
        public enum IntakeState {
            RUNNING,
            STOPPED
        }
    }
}