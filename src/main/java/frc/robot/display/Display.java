package frc.robot.display;
 
import frc.robot.subsystems.swerve.Swerve;
 
import org.frcteam6941.looper.Updatable;
 
public class Display implements Updatable {
    // Singleton instance of Display
    private static Display instance;
    FieldView fieldView;
    Swerve swerve;
 
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
                swerve.getLocalizer().getLatestPose(),
                swerve.getLocalizer().getPredictedPose(0.02)
        );
    }
}