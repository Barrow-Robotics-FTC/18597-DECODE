package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.paths.PathChain;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import static org.firstinspires.ftc.teamcode.Constants.TeleOpConstants;
import org.firstinspires.ftc.teamcode.Constants.Mode;
import org.firstinspires.ftc.teamcode.Constants.MovementVectors;
import org.firstinspires.ftc.teamcode.pedro.PedroConstants;
import org.firstinspires.ftc.teamcode.Robot;

public class Drivetrain {
    public Follower follower;
    private boolean lastState;

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
     * Set the starting pose for Pedro Pathing
     * Automatically calls `update()` to apply the new starting pose
     *
     * @param pose The starting pose to set
     */
    public void setStartingPose(Pose pose) {
        follower.setStartingPose(pose); // Set the starting pose in Pedro Pathing
        follower.update(); // Update Pedro Pathing to apply the new starting pose
    }

    /**
     * Set the current pose for Pedro Pathing
     *
     * @param pose The current pose to set
     */
    public void setPose(Pose pose) {
        follower.setPose(pose); // Set the robot's pose in Pedro Pathing
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
    public void holdPose(Pose pose) {
        follower.holdPoint(pose); // Hold the provided pose
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
            if (lastState && !isDriving()) {
                startTeleOp(); // Start the TeleOp again
            }
            lastState = isDriving(); // Update the last state
        }
    }
}