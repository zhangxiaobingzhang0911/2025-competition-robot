package frc.robot.display;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.swerve.Swerve;
import lombok.Setter;
import org.frcteam6941.looper.Updatable;

public class Display implements Updatable {
    // Singleton instance of Display
    private static Display instance;
    FieldView fieldView;
    Swerve swerve;
    @Setter
    private Pose2d aimingTarget = new Pose2d();

    // Private constructor to prevent instantiation
    private Display() {
        swerve = Swerve.getInstance();
        fieldView = new FieldView();
    }

    // Returns the singleton instance of Display, creating it if necessary
    public static Display getInstance() {
        if (instance == null) {
            instance = new Display();
        }
        return instance;
    }

    // Updates the display with the latest and predicted poses from the swerve subsystem
    @Override
    public void update(double time, double dt) {
        fieldView.update(
                swerve.getLocalizer().getCoarseFieldPose(Timer.getFPGATimestamp()),
                swerve.getLocalizer().getPredictedPose(0.02),
                aimingTarget
        );
    }
}