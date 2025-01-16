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
import frc.robot.utils.AllianceFlipUtil;

import lombok.Getter;
import lombok.Synchronized;

import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

public class AutoActions {
    private final static Swerve swerve = Swerve.getInstance();

    @Getter
    private static final Map<String, Command> eventMap = new HashMap<>();

    static {
    }


    // private final static FullAutoBuilder autoBuilder = new FullAutoBuilder(
    //         swerve,
    //         swerve::resetPose,
    //         eventMap
    // );

    //load Traj from Path and flip according to alliance
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


    //todo: Think Choreo
    // private PathPlannerPath getPath(String name){
    //     return PathPlannerPath.fromChoreoTrajectory(null);
    // }


    // @Synchronized
    // public static List<PathPlannerTrajectory> getTrajectoryGroup(String name, PathConstraints constraints) {
    //     return PathPlanner.loadPathGroup(name, constraints);
    // }


    // ToDo: think about following with events

    // @Synchronized
    // public static Command followTrajectoryWithEvents(PathPlannerTrajectory trajectory, boolean lockAngle) {
    //     return new FollowTrajectoryWithEvents(swerve, trajectory, lockAngle, true, eventMap);
    // }

    @Synchronized
    public static Command followTrajectory(PathPlannerTrajectory trajectory, boolean angleLock, boolean requiredOnTarget) {
        return new FollowTrajectory(swerve, trajectory, angleLock, requiredOnTarget);
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

    // @Synchronized
    // public static Command fullAuto(PathPlannerTrajectory trajectory) {
    //     return autoBuilder.fullAuto(trajectory);
    // }

    // @Synchronized
    // public static Command fullAuto(List<PathPlannerTrajectory> trajectories) {
    //     return new WaitUntilCommand(
    //             () -> (intaker.isHomed() && hood.isCalibrated()) || !Constants.IS_REAL
    //     ).andThen(
    //             autoBuilder.fullAuto(trajectories)
    //     );
    // }
}
