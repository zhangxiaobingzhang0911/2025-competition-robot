package frc.robot.auto.basics;

import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;

import java.util.Map;

public class FollowTrajectoryWithEvents extends FollowPathWithEvents {
    Swerve swerve;
    public FollowTrajectoryWithEvents(Swerve swerve, PathPlannerTrajectory trajectory, boolean lockAngle, boolean requiredOnTarget, Map<String, Command> eventMap) {
        super(new FollowTrajectory(swerve, trajectory, lockAngle, requiredOnTarget), trajectory.getMarkers(), eventMap);
        this.swerve = swerve;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopMovement();
    }
}
