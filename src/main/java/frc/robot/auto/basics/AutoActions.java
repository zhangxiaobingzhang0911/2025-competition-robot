package frc.robot.auto.basics;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.swerve.Swerve;
import lombok.Getter;
import lombok.Synchronized;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.AllianceFlipUtil;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

public class AutoActions {
    private final static Swerve swerve = Swerve.getInstance();

    @Getter
    private static final Map<String, Command> eventMap = new HashMap<>();

    @Synchronized
    public static PathPlannerTrajectory getTrajectory(String name) throws FileVersionException, IOException, ParseException {
        if (AllianceFlipUtil.shouldFlip()) {
            return new PathPlannerTrajectory(
                    PathPlannerPath.fromPathFile(name).flipPath(),
                    swerve.getChassisSpeeds(),
                    swerve.getLocalizer().getLatestPose().getRotation(), swerve.getAutoConfig()
            );
        } else {
            return new PathPlannerTrajectory(
                    PathPlannerPath.fromPathFile(name),
                    swerve.getChassisSpeeds(),
                    swerve.getLocalizer().getLatestPose().getRotation(), swerve.getAutoConfig()
            );
        }
    }

    @Synchronized
    public static Command followTrajectory(PathPlannerTrajectory trajectory, boolean angleLock, boolean requiredOnTarget) {
        return new FollowTrajectory(swerve, trajectory, angleLock, requiredOnTarget);
    }

    @Synchronized
    public static Command followTrajectoryVolatile(PathPlannerTrajectory trajectory, boolean angleLock, boolean requiredOnTarget) {
        return new SequentialCommandGroup(
                resetOdometryToTrajectoryStart(trajectory),
                followTrajectory(trajectory, angleLock, requiredOnTarget)
        );
    }

    @Synchronized
    public static Command resetOdometryToTrajectoryStart(PathPlannerTrajectory trajectory) {
        return new InstantCommand(() -> swerve.resetPose(trajectory.getInitialPose()));
    }

    @Synchronized
    public static Command resetOdometry(Pose2d startingPose) {
        return new InstantCommand(() -> swerve.resetPose(startingPose));
    }

    @Synchronized
    public static Command waitFor(double seconds) {
        return new WaitCommand(seconds);
    }

    @Synchronized
    public static Command print(String message) {
        return new PrintCommand(message);
    }
}