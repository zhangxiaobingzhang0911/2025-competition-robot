package frc.robot.auto.basics;


import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;


public class FollowTrajectory extends Command {
    PathPlannerTrajectory trajectory;
    boolean angleLock;
    boolean onTarget;

    Swerve mDrivebase;

    public FollowTrajectory(Swerve mDrivebase, PathPlannerTrajectory trajectory, boolean angleLock, boolean requiredOnTarget) {
        this.mDrivebase = mDrivebase;
        this.trajectory = trajectory;
        this.angleLock = angleLock;
        this.onTarget = requiredOnTarget;
    }

    @Override
    public void initialize() {
        mDrivebase.follow(trajectory, angleLock, onTarget);
    }


    @Override
    public void end(boolean interrupted) {
        mDrivebase.stopMovement();
    }

    @Override
    public boolean isFinished() {
        return !this.mDrivebase.getFollower().isPathFollowing();
    }
}
