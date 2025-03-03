package frc.robot.auto.basics;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.json.simple.parser.ParseException;

import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import static frc.robot.drivers.DestinationSupplier.elevatorSetpoint.L4;

public class AutoFile {
    private final AutoActions autoActions;
    private final Map<String, PathPlannerPath> autoPaths = new HashMap<>();

    public AutoFile(AutoActions autoActions) {
        this.autoActions = autoActions;
        initializeAutoPaths();
    }

    private void initializeAutoPaths() {
        File[] files = new File(Filesystem.getDeployDirectory(), "pathplanner/paths").listFiles();
        assert files != null;
        for (File file : files) {
            try {
                // path files without extension
                PathPlannerPath path = PathPlannerPath.fromPathFile(file.getName().replaceFirst("[.][^.]+$", ""));
                autoPaths.put(path.name, path);
            } catch (IOException | ParseException e) {
                throw new IllegalArgumentException("Failed to parse path file: " + file.getName(), e);
            }
        }
    }

    private PathPlannerPath getAutoPath(String path) {
        assert autoPaths.containsKey(path);
        return autoPaths.get(path);
    }

    public Command runAuto(String autoName) {
        return switch (autoName) {
            case "4CoralUp" -> build4CoralUp();
            case "AutoAimingTest" -> buildAutoAimingTest();
            default -> throw new IllegalArgumentException("No corresponding auto named " + autoName);
        };
    }

    private Command build4CoralUp() {
        return new SequentialCommandGroup(
                //autoActions.initializeVision(),
                autoActions.setL4(),
                autoActions.followPath(getAutoPath("left1"), true, true, true),
                autoActions.putCoral(),
                autoActions.followPath(getAutoPath("left2"), true, true, false),
                autoActions.followPath(getAutoPath("left3"), true, true, false),
                autoActions.followPath(getAutoPath("left4"), true, true, false),
                autoActions.followPath(getAutoPath("left5"), true, true, false),
                new WaitCommand(0.2),
                autoActions.shootCoral()
//                autoActions.followPath(getAutoPath("P3-I1"), true, true, false),
//                autoActions.followPath(getAutoPath("I1-P2-1"), true, true, false),
//                autoActions.followPath(getAutoPath("P2-1-I2"), true, true, false),
//                autoActions.followPath(getAutoPath("I2-P1-2"), true, true, false),
//                autoActions.followPath(getAutoPath("P1-2-I3"), true, true, false),
//                autoActions.followPath(getAutoPath("I3-P1-1"), true, true, false)
        );
    }

    private Command buildAutoAimingTest() {
        return new SequentialCommandGroup(
                //autoActions.initializeVision(),
                autoActions.followPath(getAutoPath("S1-H"), true, true, false),
                autoActions.AutoAimShoot(L4, 'J'),
                autoActions.followPath(getAutoPath("J-I1"), true, true, false),
                autoActions.followPath(getAutoPath("I1-A"), true, true, false),
                autoActions.AutoAimShoot(L4, 'L')
        );
    }
}
