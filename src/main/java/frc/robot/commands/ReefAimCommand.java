package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.swerve.Swerve;

// FIXME Needed?
public class ReefAimCommand extends Command {
    private final AprilTagVision aprilTagVision;
    private final Swerve swerve;
    private final int tagID;
    private final boolean rightReef; // true if shooting right reef
    private Pose2d robotPose;
    private Pose2d tagPose;
    private Pose2d destinationPose;
    private Transform2d transform;
    private boolean isFinished = false;

    public ReefAimCommand(AprilTagVision aprilTagVision, int tagID, boolean rightReef) {
        this.aprilTagVision = aprilTagVision;
        this.swerve = Swerve.getInstance();
        addRequirements(this.aprilTagVision, this.swerve);
        this.tagID = tagID;
        this.rightReef = rightReef;
    }

    @Override
    public void initialize() {
        this.robotPose = this.aprilTagVision.getRobotPose3d().toPose2d();
        this.tagPose = this.aprilTagVision.getAllTagPoses().get(this.tagID).toPose2d();
        if (this.rightReef) {
            this.destinationPose = this.tagPose.transformBy(RobotConstants.ReefAimConstants.tagRightToRobot);
        } else {
            this.destinationPose = this.tagPose.transformBy(RobotConstants.ReefAimConstants.tagLeftToRobot);
        }
        this.transform = this.destinationPose.minus(this.robotPose);
    }

    @Override
    public void execute() {
        this.swerve.drive(this.transform.getTranslation(),
                MathUtil.angleModulus(this.transform.getRotation().getRadians()), false, false);
        this.isFinished = true;
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run
        return this.isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        this.swerve.brake();
    }
}
