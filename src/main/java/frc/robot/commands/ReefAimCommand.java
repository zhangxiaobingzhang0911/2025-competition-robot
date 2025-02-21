package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.RobotConstants;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.swerve.Swerve;
import org.littletonrobotics.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;

public class ReefAimCommand extends Command {
    private final Swerve swerve = Swerve.getInstance();
    private int tagID;
    private boolean rightReef; // true if shooting right reef
    private final PIDController xPID = new PIDController(
            RobotConstants.SwerveConstants.AimGainsClass.AIM_KP.get(),
            RobotConstants.SwerveConstants.AimGainsClass.AIM_KI.get(),
            RobotConstants.SwerveConstants.AimGainsClass.AIM_KD.get());
    private final PIDController yPID = new PIDController(
            RobotConstants.SwerveConstants.AimGainsClass.AIM_KP.get(),
            RobotConstants.SwerveConstants.AimGainsClass.AIM_KI.get(),
            RobotConstants.SwerveConstants.AimGainsClass.AIM_KD.get());
    private Pose2d robotPose;
    private Pose2d tagPose;
    private Pose2d destinationPose;
    private Translation2d translationalVelocity;

    // Constructor for ReefAimCommand
    public ReefAimCommand(AprilTagVision aprilTagVision) {

        addRequirements(this.swerve);

    }

    @Override
    public void initialize() {

        xPID.setTolerance(0.02);

        yPID.setTolerance(0.02);


    }

    @Override
    public void execute() {
        if (RobotConstants.TUNING) {
            xPID.setP(RobotConstants.SwerveConstants.AimGainsClass.AIM_KP.get());
            xPID.setI(RobotConstants.SwerveConstants.AimGainsClass.AIM_KI.get());
            xPID.setD(RobotConstants.SwerveConstants.AimGainsClass.AIM_KD.get());
            yPID.setP(RobotConstants.SwerveConstants.AimGainsClass.AIM_KP.get());
            yPID.setI(RobotConstants.SwerveConstants.AimGainsClass.AIM_KI.get());
            yPID.setD(RobotConstants.SwerveConstants.AimGainsClass.AIM_KD.get());
        }

        rightReef = DestinationSupplier.getInstance().getCurrentBranch();
        //testing delete when not using sim
        tagPose = FieldConstants.defaultAprilTagType.getLayout().getTagPose(17).get().toPose2d();
        //tagPose = vision.getClosestTagPose().toPose2d();
        //[fixme]IMMINANT, use coarsed field pose

        if (rightReef) {
            destinationPose = tagPose.transformBy(RobotConstants.ReefAimConstants.tagRightToRobot);
        } else {
            destinationPose = tagPose.transformBy(RobotConstants.ReefAimConstants.tagLeftToRobot);
        }
        xPID.setSetpoint(destinationPose.getTranslation().getX());
        yPID.setSetpoint(destinationPose.getTranslation().getY());
        swerve.setLockHeading(true);
        swerve.setHeadingTarget(destinationPose.getRotation().getDegrees() - 180.0);
        robotPose = swerve.getLocalizer().getCoarseFieldPose(0);
        translationalVelocity = new Translation2d(xPID.calculate(robotPose.getX() - 0.45), yPID.calculate(robotPose.getY()));
        swerve.drive(AllianceFlipUtil.shouldFlip() ? translationalVelocity.unaryMinus() : translationalVelocity, 0.0, true, false);


        Logger.recordOutput("ReefAimCommand/tagID", tagID);
        Logger.recordOutput("ReefAimCommand/rightReef", rightReef);
        Logger.recordOutput("ReefAimCommand/destinationPose", destinationPose.toString());
        Logger.recordOutput("ReefAimCommand/xFinished", xPID.atSetpoint());
        Logger.recordOutput("ReefAimCommand/yFinished", yPID.atSetpoint());
        Logger.recordOutput("ReefAimCommand/omegaFinished", Swerve.getInstance().aimingReady(1,2.14));
        Logger.recordOutput("ReefAimCommand/RobotPose", robotPose.toString());
        Logger.recordOutput("ReefAimCommand/translationalVelocity", translationalVelocity.toString());
    }

    @Override
    public boolean isFinished() {
        return (xPID.atSetpoint() && yPID.atSetpoint() && Swerve.getInstance().aimingReady(1,2.14));
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setLockHeading(false);
    }

    @Override

    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}