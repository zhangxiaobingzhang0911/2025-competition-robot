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
    private final AutoActions autoActions;

    public AutoFile(AutoActions autoActions) {
        this.autoActions = autoActions;
    }

    private List<PathPlannerPath> getAutoPaths(String fileName) {
        try {
            return PathPlannerAuto.getPathGroupFromAutoFile(fileName);
        } catch (IOException | ParseException e) {
            throw new IllegalArgumentException("Failed to parse auto file: " + fileName, e);
        }
    }

    public Command runAuto(String fileName, boolean angleLock, boolean requiredOnTarget, boolean resetOdometry) {
//        List<PathPlannerPath> paths = getAutoPaths(fileName);
//        SequentialCommandGroup group = new SequentialCommandGroup();
//        for (int i = 0; i < paths.size(); i++) {
//            PathPlannerPath path = paths.get(i);
//            // reset odometry only at auto start
//            group.addCommands(new FollowPath(Swerve.getInstance(), path, angleLock, requiredOnTarget, resetOdometry && i == 0));
//        }
//        return group;
//    }
        return buildTestAuto(fileName);
    }

    Command buildTestAuto(String fileName) {
        SequentialCommandGroup testAuto = new SequentialCommandGroup();
        List<PathPlannerPath> paths = getAutoPaths(fileName);
        PathPlannerPath path = paths.get(0);
        testAuto.addCommands(autoActions.preShoot());
        testAuto.addCommands(new FollowPath(Swerve.getInstance(), path, true, true, true));
        testAuto.addCommands(autoActions.raiseElevator());
        testAuto.addCommands(autoActions.shootCoral());
        return testAuto;
    }
}
