package frc.robot.display;
 
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 
public class FieldView {
    // Represents the field view on the SmartDashboard
    private final Field2d mField2d = new Field2d();
    // Subscribes to a NetworkTable topic for receiving double array data
    DoubleArraySubscriber doubleArray = NetworkTableInstance.getDefault().
                                                                        getDoubleArrayTopic("/orangePi5Plus/output/demo_observations/").
                                                                        subscribe(new double[]{}, PubSubOption.keepDuplicates(true), 
                                                                        PubSubOption.sendAll(true));
 
    // Constructor for FieldView that initializes the Field2d instance on the SmartDashboard
    public FieldView() {
        SmartDashboard.putData(mField2d);
    }
 
    // Method intended for drawing the field, currently empty
    private void drawField() {
    }
 
    // Updates the field view with the current robot pose and a ghost pose, also processes incoming double array data
    public void update(Pose2d pose, Pose2d ghost) {
        drawField();
 
        mField2d.setRobotPose(pose);
        mField2d.getObject("Ghost").setPose(ghost);
        if (doubleArray.exists()) {
            if (doubleArray.get().length > 3) {
                System.out.println(new Pose2d(
                        doubleArray.get()[1],
                        doubleArray.get()[2],
                        Rotation2d.fromDegrees(doubleArray.get()[3]))
                );
            }
        }
    }
}