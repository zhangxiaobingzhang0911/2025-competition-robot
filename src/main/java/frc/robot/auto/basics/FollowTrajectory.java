package frc.robot.auto.basics;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;

// A command that makes the robot follow a predefined trajectory using the Swerve drivebase
public class FollowTrajectory extends Command {
    PathPlannerTrajectory trajectory;
    boolean angleLock;
    boolean onTarget;

    Swerve mDrivebase;

    // Constructor for FollowTrajectory command, initializes the drivebase, trajectory, angle lock and on target requirements
    public FollowTrajectory(Swerve mDrivebase, PathPlannerTrajectory trajectory, boolean angleLock, boolean requiredOnTarget) {
        this.mDrivebase = mDrivebase;
        this.trajectory = trajectory;
        this.angleLock = angleLock;
        this.onTarget = requiredOnTarget;
    }

    // Called once when the command is scheduled
    @Override
    public void initialize() {
        mDrivebase.follow(trajectory, angleLock, onTarget);
    }

    // Called once when the command ends or is interrupted
    @Override
    public void end(boolean interrupted) {
        mDrivebase.stopMovement();
    }

    // Returns true if the command is finished, false otherwise
    @Override
    public boolean isFinished() {
        return !this.mDrivebase.getFollower().isPathFollowing();
    }
}