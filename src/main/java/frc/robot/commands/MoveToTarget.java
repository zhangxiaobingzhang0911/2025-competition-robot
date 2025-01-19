package frc.robot.commands;
 
import com.team254.lib.geometry.Twist2d;
 
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
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
        // Constructor initializes the swerve drive and AprilTag vision subsystems, and adds them as command requirements
        this.Swerve = Swerve;
        this.aprilTagVision = aprilTagVision;
        addRequirements(Swerve, aprilTagVision);
    }
 
    @Override
    public void initialize() {
        // Initialize sets the target pose to the closest pose detected by AprilTag vision
        System.out.println("Starting MoveToTarget command.");
        this.targetPose = aprilTagVision.getClosestTagPose();
    }
 
    @Override
    public void execute() {
        // Execute calculates the difference between the target and current pose, and drives the swerve towards the target
        Twist2d currentPose = Swerve.getChassisTwist();
        double deltaX = targetPose.getX() - currentPose.dx;
        double deltaY = targetPose.getY() - currentPose.dy;
        double distance = Math.hypot(deltaX, deltaY);
        Translation2d translation = new Translation2d(deltaX, deltaY);
 
        double angleToTarget = Math.atan2(deltaY, deltaX);
 
        if (distance <= TARGET_RADIUS) {
            // Stop the swerve drive if the robot is within the target radius
            Swerve.stop();
            return;
        }
 
        Swerve.drive(translation, angleToTarget, true, false);
    }
 
    @Override
    public boolean isFinished() {
        // Determines if the command should end based on the robot's proximity to the target
        Twist2d currentPose = Swerve.getChassisTwist();
        double deltaX = targetPose.getX() - currentPose.dx;
        double deltaY = targetPose.getY() - currentPose.dy;
        return Math.hypot(deltaX, deltaY) <= TARGET_RADIUS;
    }
 
    @Override
    public void end(boolean interrupted) {
        // Ends the command by stopping the swerve drive and prints a message
        Swerve.stop();
        System.out.println("MoveToTarget command ended.");
    }
}