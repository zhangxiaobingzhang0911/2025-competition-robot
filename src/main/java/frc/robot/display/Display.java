package frc.robot.display;

import frc.robot.subsystems.swerve.Swerve;
import org.frcteam6941.Looper.Updatable;

public class Display implements Updatable {
    private static Display instance;
    FieldView fieldView;
    Swerve swerve;

    private Display() {
        swerve = Swerve.getInstance();
        fieldView = new FieldView();
    }

    public static Display getInstance() {
        if (instance == null) {
            instance = new Display();
        }
        return instance;
    }

    @Override
    public void update(double time, double dt) {
        fieldView.update(
                swerve.getLocalizer().getLatestPose(),
                swerve.getLocalizer().getPredictedPose(0.02)
        );
    }
}
