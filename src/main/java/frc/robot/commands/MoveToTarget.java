package frc.robot.commands;

import com.team254.lib.geometry.Twist2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.swerve.Swerve;

public class MoveToTarget extends Command {

    private final Swerve Swerve;
    private Pose3d targetPose;
    private final AprilTagVision aprilTagVision;
    //private static final double MAX_SPEED = 1.0; // Maximum speed in meters per second
    private static final double TARGET_RADIUS = 0.2; // 20 cm radius for stopping

    public MoveToTarget(Swerve Swerve, AprilTagVision aprilTagVision) {
        this.Swerve = Swerve;
        this.aprilTagVision = aprilTagVision;
        addRequirements(Swerve, aprilTagVision);
    }

    @Override
    public void initialize() {
        System.out.println("Starting MoveToTarget command.");
        this.targetPose = aprilTagVision.getClosestTagPose();
    }

    @Override
    public void execute() {
        // Get the robot's current pose
        Twist2d currentPose = Swerve.getChassisTwist();
        // Calculate the difference in position and orientation
        double deltaX = targetPose.getX() - currentPose.dx;
        double deltaY = targetPose.getY() - currentPose.dy;
        double distance = Math.hypot(deltaX, deltaY);
        Translation2d translation = new Translation2d(deltaX, deltaY);

        // Calculate the angle to the target
        double angleToTarget = Math.atan2(deltaY, deltaX);

        // Stop if within the target radius
        if (distance <= TARGET_RADIUS) {
            Swerve.stop();
            return;
        }



        Swerve.drive(translation, angleToTarget, true, false);
    }

    @Override
    public boolean isFinished() {
        // Finish the command if the robot is within the target radius
        Twist2d currentPose = Swerve.getChassisTwist();
        double deltaX = targetPose.getX() - currentPose.dx;
        double deltaY = targetPose.getY() - currentPose.dy;
        return Math.hypot(deltaX, deltaY) <= TARGET_RADIUS;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot
        Swerve.stop();
        System.out.println("MoveToTarget command ended.");
    }
}
