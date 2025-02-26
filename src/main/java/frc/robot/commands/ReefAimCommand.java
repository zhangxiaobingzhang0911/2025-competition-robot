package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.display.Display;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.subsystems.swerve.Swerve;
import org.littletonrobotics.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;

import static frc.robot.RobotConstants.ReefAimConstants;

public class ReefAimCommand extends Command {
    private final Swerve swerve = Swerve.getInstance();
    private final ProfiledPIDController xPID = new ProfiledPIDController(
            RobotConstants.SwerveConstants.AimGainsClass.AIM_KP.get(),
            RobotConstants.SwerveConstants.AimGainsClass.AIM_KI.get(),
            RobotConstants.SwerveConstants.AimGainsClass.AIM_KD.get(),
            new TrapezoidProfile.Constraints(
                    ReefAimConstants.MAX_AIMING_SPEED.magnitude(),
                    ReefAimConstants.MAX_AIMING_ACCELERATION.magnitude()));
    private final ProfiledPIDController yPID = new ProfiledPIDController(
            RobotConstants.SwerveConstants.AimGainsClass.AIM_KP.get(),
            RobotConstants.SwerveConstants.AimGainsClass.AIM_KI.get(),
            RobotConstants.SwerveConstants.AimGainsClass.AIM_KD.get(),
            new TrapezoidProfile.Constraints(
                    ReefAimConstants.MAX_AIMING_SPEED.magnitude(),
                    ReefAimConstants.MAX_AIMING_ACCELERATION.magnitude()));
    private final BooleanSupplier stop;

    private boolean rightReef; // true if shooting right reef
    private boolean xFinished = false;
    private boolean yFinished = false;
    private boolean omegaFinished = false;
    private Pose2d robotPose, tagPose, destinationPose;
    private Translation2d translationalVelocity;


    public ReefAimCommand(BooleanSupplier stop) {
        addRequirements(this.swerve);
        this.stop = stop;
    }

    @Override
    public void initialize() {
        // Calculate destination
        robotPose = swerve.getLocalizer().getCoarseFieldPose(0);
        tagPose = DestinationSupplier.getInstance().getNearestTag(robotPose);
        // PID init
        xPID.reset(robotPose.getX(), swerve.getLocalizer().getMeasuredVelocity().getX());
        yPID.reset(robotPose.getY(), swerve.getLocalizer().getMeasuredVelocity().getY());
    }

    @Override
    public void execute() {
        if (RobotConstants.TUNING) {
            xPID.setPID(RobotConstants.SwerveConstants.AimGainsClass.AIM_KP.get(),
                    RobotConstants.SwerveConstants.AimGainsClass.AIM_KI.get(),
                    RobotConstants.SwerveConstants.AimGainsClass.AIM_KD.get());
            yPID.setPID(RobotConstants.SwerveConstants.AimGainsClass.AIM_KP.get(),
                    RobotConstants.SwerveConstants.AimGainsClass.AIM_KI.get(),
                    RobotConstants.SwerveConstants.AimGainsClass.AIM_KD.get());
        }

        robotPose = swerve.getLocalizer().getCoarseFieldPose(0);
        rightReef = DestinationSupplier.getInstance().getCurrentBranch();
        destinationPose = DestinationSupplier.getDriveTarget(robotPose, tagPose, rightReef);

        xPID.setGoal(destinationPose.getTranslation().getX());
        yPID.setGoal(destinationPose.getTranslation().getY());
        swerve.setLockHeading(true);
        swerve.setHeadingTarget(destinationPose.getRotation().getDegrees() - 180.0);
        translationalVelocity =
                AllianceFlipUtil.shouldFlip() ?
                        new Translation2d(-xPID.calculate(robotPose.getX()), -yPID.calculate(robotPose.getY())) :
                        new Translation2d(xPID.calculate(robotPose.getX()), yPID.calculate(robotPose.getY()));
        swerve.drive(translationalVelocity, 0.0, true, false);
        Display.getInstance().setAimingTarget(destinationPose);
        Logger.recordOutput("ReefAimCommand/tagPose", tagPose);
        Logger.recordOutput("ReefAimCommand/destinationPose", destinationPose);
        Logger.recordOutput("ReefAimCommand/translationalVelocity", translationalVelocity);
    }

    @Override
    public boolean isFinished() {
        xFinished = Math.abs(robotPose.getX() - destinationPose.getX()) < ReefAimConstants.X_TOLERANCE.magnitude();
        yFinished = Math.abs(robotPose.getY() - destinationPose.getY()) < ReefAimConstants.Y_TOLERANCE.magnitude();
        omegaFinished = Swerve.getInstance().aimingReady(0.5, 2.14);
        Logger.recordOutput("ReefAimCommand/xFinished", xFinished);
        Logger.recordOutput("ReefAimCommand/yFinished", yFinished);
        Logger.recordOutput("ReefAimCommand/omegaFinished", omegaFinished);
        Logger.recordOutput("ReefAimCommand/emergencyStopped", stop.getAsBoolean());
        return (xFinished && yFinished && omegaFinished) || stop.getAsBoolean();

    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(), 0.0, true, false);
        swerve.setLockHeading(false);
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}