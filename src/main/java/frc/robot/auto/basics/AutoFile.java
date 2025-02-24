package frc.robot.auto.basics;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.Swerve;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.List;

public class AutoFile {
    private static List<PathPlannerPath> getAutoPaths(String fileName) {
        try {
            return PathPlannerAuto.getPathGroupFromAutoFile(fileName);
        } catch (IOException | ParseException e) {
            throw new IllegalArgumentException("Failed to parse auto file: " + fileName, e);
        }
    }

    public static Command runAuto(String fileName, boolean angleLock, boolean requiredOnTarget, boolean resetOdometry) {
        List<PathPlannerPath> paths = getAutoPaths(fileName);
        SequentialCommandGroup group = new SequentialCommandGroup();
        for (int i = 0; i < paths.size(); i++) {
            PathPlannerPath path = paths.get(i);
            // reset odometry only at auto start
            group.addCommands(new FollowPath(Swerve.getInstance(), path, angleLock, requiredOnTarget, resetOdometry && i == 0));
        }
        return group;
    }
}
