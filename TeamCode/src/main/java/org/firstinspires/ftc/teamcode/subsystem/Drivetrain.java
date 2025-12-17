package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.paths.PathChain;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import static org.firstinspires.ftc.teamcode.Constants.TeleOpConstants;
import static org.firstinspires.ftc.teamcode.Constants.PATH_SLOW_DOWN_SPEED;
import static org.firstinspires.ftc.teamcode.Constants.WALL_SLOW_DOWN_DISTANCE;
import static org.firstinspires.ftc.teamcode.Constants.WALL_SLOW_DOWN_SPEED;
import org.firstinspires.ftc.teamcode.Constants.Mode;
import org.firstinspires.ftc.teamcode.Constants.MovementVectors;
import org.firstinspires.ftc.teamcode.pedro.PedroConstants;
import org.firstinspires.ftc.teamcode.Robot;

public class Drivetrain {
    public Follower follower;
    private boolean lastState;
    private boolean holdingPose;

    public Drivetrain(Robot robot, HardwareMap hardwareMap) {
        this.follower = PedroConstants.createFollower(hardwareMap);

        // Start Pedro Pathing in TeleOp mode if applicable
        if (robot.mode == Mode.TELEOP) {
            startTeleOp();
        }
    }

    private void startTeleOp() {
        follower.startTeleOpDrive(TeleOpConstants.BRAKE_MODE);
    }

    /**
     * Set the movement vectors for TeleOp driving
     *
     * @param movementVectors The movement vectors to set
     */
    public void setMovementVectors(MovementVectors movementVectors) {
        if (!follower.isBusy()) { // Make sure we are not auto-driving
            // Set TeleOp drive with the provided movement vectors
            follower.setTeleOpDrive(movementVectors.forward, movementVectors.strafe,
                    movementVectors.turn, TeleOpConstants.ROBOT_CENTRIC
            );
        }
    }

    /**
     * Set the current pose for Pedro Pathing
     *
     * @param pose The current pose to set
     */
    public void setPose(Pose pose) {
        follower.setPose(pose); // Set the robot's pose in Pedro Pathing
        follower.update(); // Update Pedro Pathing to apply the new pose
    }

    /**
     * Follow a path chain using Pedro Pathing
     *
     * @param speed The speed to follow the path at
     * @param pathChain The path chain to follow
     */
    public void followPath(PathChain pathChain, double speed) {
        follower.followPath(pathChain, speed, true);// Follow the provided path chain
    }

    /**
     * Follow a path chain using Pedro Pathing
     *
     * @param pathChain The path chain to follow
     */
    public void followPath(PathChain pathChain) {
        followPath(pathChain, PedroConstants.driveConstants.maxPower); // Follow the path at default speed
    }

    /**
     * Hold the robot at a specific pose using Pedro Pathing
     *
     * @param pose The pose to hold
     */
    public void holdPoint(Pose pose) {
        follower.holdPoint(pose); // Hold the provided pose
        holdingPose = true; // We only need to set this when we are deliberately holding a pose
    }

    /**
     * Stop an ongoing hold point action
     * Only necessary if the hold point was set deliberately using `holdPose()`
     */
    public void stopHoldPoint() {
        if (holdingPose) {
            follower.breakFollowing(); // Stop holding the point
            holdingPose = false; // Reset the holding pose flag
        }
    }

    /**
     * Set the maximum power for Pedro Pathing movements
     *
     * @param maxPower The maximum power to set (0.0 to 1.0)
     */
    public void setMaxPower(double maxPower) {
        follower.setMaxPower(maxPower); // Set the maximum power for Pedro Pathing
    }

    /**
     * Set the max power near the end of the path for smoother stopping
     */
    public void slowDownForPathEnd() {
        follower.setMaxPower(PATH_SLOW_DOWN_SPEED); // Set max power to slow down near path end
    }

    /**
     * Get the current pose of the robot
     *
     * @return Current pose of the robot
     */
    public Pose getPose() {
        return follower.getPose(); // Get the current pose from Pedro Pathing
    }

    /**
     * Check if the drivetrain is currently autonomously driving
     *
     * @return True if autonomously driving, false otherwise
     */
    public boolean isDriving() {
        return follower.isBusy();
    }

    public void stop() {
        if (isDriving()) {
            follower.breakFollowing(); // Stop any ongoing path following
        }
        follower.holdPoint(getPose()); // Hold the current position
    }

    public void update(Robot robot) {
        follower.update(); // Update Pedro Pathing

        if (robot.mode == Mode.TELEOP) {
            // Check if an autonomous drive has just completed
            if (lastState && !isDriving() && !holdingPose) {
                startTeleOp(); // Start the TeleOp again
            }
            lastState = isDriving() || holdingPose; // Update the last state
        }

        /* Needs to be tested before enabling
        // Check if we're near a wall and adjust max power if necessary
        if (getPose().getX() < WALL_SLOW_DOWN_DISTANCE ||
                getPose().getX() > 144 - WALL_SLOW_DOWN_DISTANCE ||
                getPose().getY() < WALL_SLOW_DOWN_DISTANCE ||
                getPose().getY() > 144 - WALL_SLOW_DOWN_DISTANCE)
        {
            setMaxPower(WALL_SLOW_DOWN_SPEED); // Slow down near the wall
        } else if (!isDriving() && !holdingPose) {
            setMaxPower(1.0); // Reset max power to full if not driving or holding
        } // Max speed will be managed during autonomous driving
        */
    }
}