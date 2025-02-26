package frc.robot.display;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lombok.Setter;

public class FieldView {
    private final Field2d mField2d = new Field2d();
    DoubleArraySubscriber doubleArray = NetworkTableInstance.getDefault().
            getDoubleArrayTopic("/orangePi5Plus/output/demo_observations/").
            subscribe(new double[]{}, PubSubOption.keepDuplicates(true),
                    PubSubOption.sendAll(true));
    @Setter
    private Pose2d aimingTarget = new Pose2d();

    public FieldView() {
        SmartDashboard.putData(mField2d);
    }

    private void drawField() {
    }

    public void update(Pose2d pose, Pose2d ghost, Pose2d aimingTarget) {
        drawField();

        mField2d.setRobotPose(pose);
        mField2d.getObject("Ghost").setPose(ghost);
        mField2d.getObject("Aiming Target").setPose(aimingTarget);
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