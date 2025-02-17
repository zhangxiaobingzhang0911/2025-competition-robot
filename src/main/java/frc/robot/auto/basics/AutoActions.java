package frc.robot.auto.basics;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.swerve.Swerve;
import lombok.Getter;
import lombok.Synchronized;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.AllianceFlipUtil;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

// Class containing static methods for creating and manipulating autonomous commands for a swerve drive robot
public class AutoActions {
    private final static Swerve swerve = Swerve.getInstance();

    // Map of event names to commands to be executed during autonomous
    @Getter
    private static final Map<String, Command> eventMap = new HashMap<>();

    static {
        // Initialization block for setting up any default commands or configurations
    }

    // Method to load a trajectory from a path and flip it according to the alliance color
    @Synchronized
    public static PathPlannerTrajectory getTrajectory(String name) throws FileVersionException, IOException, ParseException {
        if (AllianceFlipUtil.shouldFlip()) {
            return new PathPlannerTrajectory(
                    PathPlannerPath.fromPathFile(name).flipPath(),
                    swerve.getChassisSpeeds(),
                    swerve.getLocalizer().getLatestPose().getRotation(), null
            );
        } else {
            return new PathPlannerTrajectory(
                    PathPlannerPath.fromPathFile(name),
                    swerve.getChassisSpeeds(),
                    swerve.getLocalizer().getLatestPose().getRotation(), null
            );
        }
    }

    // Method to follow a given trajectory
    @Synchronized
    public static Command followTrajectory(PathPlannerTrajectory trajectory, boolean angleLock, boolean requiredOnTarget) {
        return new FollowTrajectory(swerve, trajectory, angleLock, requiredOnTarget);
    }

    // Method to reset the robot's odometry to a specified starting pose
    @Synchronized
    public static Command resetOdometry(Pose2d startingPose) {
        return new InstantCommand(() -> swerve.resetPose(startingPose));
    }

    // Method to create a command that waits for a specified number of seconds
    @Synchronized
    public static Command waitFor(double seconds) {
        return new WaitCommand(seconds);
    }

    // Method to create a command that prints a specified message
    @Synchronized
    public static Command print(String message) {
        return new PrintCommand(message);
    }
}