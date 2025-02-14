package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.swerve.Swerve;

// FIXME Needed?
public class ReefAimNearest extends Command {
    private final AprilTagVision aprilTagVision;
    private final Swerve swerve;
    private final boolean rightReef;
    private Pose2d robotPose;
    private Pose2d tagPose;
    private Pose2d destinationPose;
    private Transform2d transform;
    private boolean isFinished = false;

    public ReefAimNearest(AprilTagVision aprilTagVision, boolean rightReef) {
        this.aprilTagVision = aprilTagVision;
        this.swerve = Swerve.getInstance();
        addRequirements(this.aprilTagVision, this.swerve);
        this.rightReef = rightReef;
    }

    @Override
    public void initialize() {
        this.robotPose = this.aprilTagVision.getRobotPose3d().toPose2d();
        this.tagPose = this.aprilTagVision.getClosestTagPose().toPose2d();
        if (this.rightReef) {
            this.destinationPose = this.tagPose.transformBy(RobotConstants.ReefAimConstants.tagRightToRobot);
        } else {
            this.destinationPose = this.tagPose.transformBy(RobotConstants.ReefAimConstants.tagLeftToRobot);
        }
        this.transform = this.destinationPose.minus(this.robotPose);
    }

    @Override
    public void execute() {
        this.swerve.drive(
                this.transform.getTranslation(), // Translation component of the transform
                MathUtil.angleModulus(this.transform.getRotation().getRadians()), // Rotation component of the transform, normalized to [0, 2π]
                false, // Whether to lock the orientation of the robot
                false  // Whether to use open loop control
        );
        this.isFinished = true;
    }

    @Override
    public boolean isFinished() {
        return this.isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        // Brake the swerve subsystem when the command ends
        this.swerve.brake();
    }
}

