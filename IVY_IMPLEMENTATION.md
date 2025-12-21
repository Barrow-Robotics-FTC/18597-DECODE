# Pedro Pathing Ivy Command Base Implementation

## Overview
This implementation adds new Autonomous and TeleOp opmodes that use Pedro Pathing's Ivy command base framework instead of traditional state machines and manual commands. The new opmodes maintain **exactly the same functionality** as the existing LM3Auto and LM3TeleOp, but use a more modern command-based architecture.

## What Was Changed

### 1. Dependencies (build.dependencies.gradle)
- Added `com.pedropathing:ivy:0.0.1-SNAPSHOT` dependency
- Added required Maven repositories:
  - `https://repo.dairy.foundation/releases`
  - `https://repo.dairy.foundation/snapshots`

### 2. New Command Infrastructure
Created in `org.firstinspires.ftc.teamcode.command` package:

#### CommandOpMode.java
- Base class for all command-based opmodes
- Integrates with Pedro Pathing's Ivy Scheduler
- Provides `schedule()` method to register commands
- Provides `reset()` method to cancel all commands
- Automatically executes scheduled commands in the `loop()` method

#### FollowPath.java
- Command wrapper for Pedro Pathing path following
- Supports configurable max power and hold-end behavior
- Integrates with the robot's drivetrain Follower
- `start()` initiates path following
- `done()` returns true when path is complete

### 3. Robot Class Enhancements (Robot.java)
Added command-based helper methods:

- **`periodic()`** - Updates all subsystems without requiring gamepad parameters (for command-based operation)
- **`launchCommand(int numArtifacts)`** - Returns a Sequential command that launches the specified number of artifacts
- **`startIntakeCommand()`** - Returns an Instant command to start the intake
- **`stopIntakeCommand()`** - Returns an Instant command to stop the intake
- **`speedUpLauncherCommand(boolean holdSpeed)`** - Returns an Instant command to speed up the launcher
- **`holdPose()`** - Returns an Instant command to hold the current pose
- **`releasePose()`** - Returns an Instant command to release the held pose

### 4. New Autonomous Opmode (LM3IvyAuto.java)

**Key Features:**
- Implements the exact same 9-artifact autonomous sequence as LM3Auto
- Uses command groups (Sequential, Infinite) instead of state machine
- Maintains alliance selection and start position selection
- Saves end pose to blackboard for TeleOp handoff
- All path following, launching, and intake operations are command-based

**Command Structure:**
```java
schedule(
    // Continuous subsystem updates
    new Infinite(() -> robot.periodic()),
    
    // Continuous telemetry
    new Infinite(() -> { /* telemetry updates */ }),
    
    // Sequential autonomous routine
    new Sequential(
        // Speed up launcher
        robot.speedUpLauncherCommand(false),
        
        // Move to scoring position
        new FollowPath(robot, path, 0.8),
        
        // Launch 3 artifacts
        robot.launchCommand(3),
        
        // ... continues with intake and scoring cycles
    )
);
```

**Autonomous Sequence (9 Artifacts):**
1. Move to scoring position → Launch 3 preloaded artifacts
2. Move to GPP → Intake artifacts → Move to score → Launch 3
3. Move to PGP → Intake artifacts → Move to score → Launch 3
4. Move to PPG → Intake artifacts

### 5. New TeleOp Opmode (LM3IvyTeleOp.java)

**Key Features:**
- Implements the exact same controls and behavior as LM3TeleOp
- Uses command-based architecture with a single Infinite command for the main loop
- Maintains all gamepad mappings and LED color feedback
- Supports alliance handoff from autonomous via blackboard

**Gamepad Controls (unchanged):**
- **Gamepad 1 (Driver):**
  - Left Stick: Translation (X/Y movement)
  - Right Stick: Rotation
  - Left Bumper: Toggle slow mode
  
- **Gamepad 2 (Operator):**
  - Left Bumper: Toggle intake
  - Right Bumper: Toggle launcher speed up
  - Left Trigger: Launch 1 artifact (auto-aligns to goal)
  - Right Trigger: Launch 3 artifacts (auto-aligns to goal)

**Command Structure:**
```java
schedule(
    new Sequential(
        new Wait(1),  // Initial delay
        new Infinite(() -> {
            robot.periodic();
            handleDriverControls();
            handleOperatorControls();
            setGamepadColors();
            // telemetry updates
        })
    )
);
```

## How to Use

### Running Ivy Autonomous:
1. Select "LM3 Ivy Auto" from the autonomous menu
2. Use gamepad buttons to select alliance (B=Red, X=Blue)
3. Use gamepad buttons to select start position (B=Goal Wall, X=Audience Wall)
4. Press START to run the autonomous

### Running Ivy TeleOp:
1. Select "LM3 Ivy TeleOp" from the TeleOp menu
2. If running after autonomous, alliance and pose are automatically loaded
3. Press START to begin driver control
4. Use gamepads as documented above

## Benefits of Command-Based Architecture

1. **Cleaner Code:** Commands are composable and reusable
2. **Better Concurrency:** Multiple command groups can run in parallel (e.g., telemetry + main sequence)
3. **Easier Testing:** Individual commands can be tested in isolation
4. **More Maintainable:** Adding new autonomous routines is straightforward
5. **Industry Standard:** Command-based is the standard architecture in modern FTC/FRC programming

## Compatibility

- The new Ivy-based opmodes are **completely separate** from the existing LM3Auto and LM3TeleOp
- Both versions can coexist in the same codebase
- Teams can switch between them by selecting the appropriate opmode on the driver station
- All existing subsystems (Launcher, Intake, Drivetrain, etc.) work with both versions

## Technical Details

### Ivy Scheduler
- Runs in the OpMode's `loop()` method via `Scheduler.getInstance().execute()`
- Manages command lifecycle (initialization, execution, completion)
- Supports command groups: Sequential (one after another), Parallel (simultaneously), etc.

### Command Types Used
- **Instant:** Executes immediately and completes (e.g., starting intake)
- **Wait:** Waits for a specified time
- **WaitUntil:** Waits until a condition is true
- **Infinite:** Runs forever (used for periodic updates)
- **Sequential:** Runs commands one after another
- **Command:** Custom command with start() and done() methods (e.g., FollowPath)

## Files Modified/Created

**Modified:**
- `build.dependencies.gradle` - Added Ivy dependency and repositories
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java` - Added command helper methods

**Created:**
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/command/CommandOpMode.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/command/FollowPath.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/LM3IvyAuto.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/LM3IvyTeleOp.java`

## Next Steps

1. **Test the Build:** Ensure the project compiles with the new Ivy dependency
2. **Test on Robot:** Run both autonomous and TeleOp on actual hardware
3. **Validate Functionality:** Confirm all features work identically to the original versions
4. **Consider Migration:** Once validated, teams can choose to exclusively use the Ivy versions
5. **Expand Commands:** Create additional reusable commands for common operations

## References

- Pedro Pathing Documentation: https://pedropathing.com
- Example repository: https://github.com/BaronClaps/22131-Decode
- Ivy GitHub: https://github.com/Pedro-Pathing/ivy
