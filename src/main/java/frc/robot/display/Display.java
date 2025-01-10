package frc.robot.display;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.SwerveSubsystem.SwerveSubsystem;
import org.frcteam6941.Looper.Updatable;

public class Display implements Updatable {
    FieldView fieldView;
    SwerveSubsystem swerveSubsystem;

    private static Display instance;

    private Display() {
        swerveSubsystem = SwerveSubsystem.getInstance();
        fieldView = new FieldView();
    }

    public static Display getInstance() {
        if (instance == null) {
            instance = new Display();
        }
        return instance;
    }

    public void setFerryLocation(Translation2d pos) {
        fieldView.setFerryLocation(pos);
    }

    @Override
    public void update(double time, double dt) {
        fieldView.update(
            swerveSubsystem.getLocalizer().getLatestPose(), 
            swerveSubsystem.getLocalizer().getPredictedPose(0.02)
        );
    }
}
