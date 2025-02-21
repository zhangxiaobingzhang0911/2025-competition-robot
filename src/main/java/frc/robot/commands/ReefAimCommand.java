package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.RobotConstants;
import frc.robot.display.Display;
import frc.robot.subsystems.swerve.Swerve;
import org.littletonrobotics.AllianceFlipUtil;

import java.util.function.BooleanSupplier;

import static frc.robot.RobotConstants.ReefAimConstants;

public class ReefAimCommand extends Command {
    private final Swerve swerve = Swerve.getInstance();
    private final int tagID;
    private final boolean rightReef; // true if shooting right reef
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
    private boolean xFinished = false;
    private boolean yFinished = false;
    private boolean omegaFinished = false;
    private Pose2d robotPose, tagPose, destinationPose;
    private Translation2d translationalVelocity;

    /**
     * @param rightReef: It always means the right reef RELATIVE TO TAG
     *                   (i.e when you are facing the tag, rightReef = true means the tag on your right is the target)
     */
    public ReefAimCommand(int tagID, boolean rightReef, BooleanSupplier stop) {
        addRequirements(this.swerve);
        this.tagID = tagID;
        this.rightReef = rightReef;
        this.stop = stop;
    }

    @Override
    public void initialize() {
        // Calculate destination
        tagPose = FieldConstants.officialAprilTagType.getLayout().getTagPose(tagID).get().toPose2d();
        SmartDashboard.putString("ReefAimCommand/tagPose", tagPose.toString());
        destinationPose = tagPose.transformBy(new Transform2d(
                new Translation2d(
                        ReefAimConstants.ROBOT_TO_PIPE.magnitude(),
                        ReefAimConstants.PIPE_TO_TAG.magnitude() * (rightReef ? 1 : -1)),
                new Rotation2d()));
        Display.getInstance().setAimingTarget(destinationPose);
        SmartDashboard.putString("ReefAimCommand/destinationPose", destinationPose.toString());

        // Swerve init
        robotPose = swerve.getLocalizer().getCoarseFieldPose(0);
        swerve.setLockHeading(true);
        swerve.setHeadingTarget(destinationPose.getRotation().getDegrees() - 180.0);

        // PID init
        xPID.setGoal(destinationPose.getTranslation().getX());
        yPID.setGoal(destinationPose.getTranslation().getY());
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
        translationalVelocity =
                AllianceFlipUtil.shouldFlip() ?
                        new Translation2d(-xPID.calculate(robotPose.getX()), -yPID.calculate(robotPose.getY())) :
                        new Translation2d(xPID.calculate(robotPose.getX()), yPID.calculate(robotPose.getY()));
        swerve.drive(translationalVelocity, 0.0, true, false);
        SmartDashboard.putString("ReefAimCommand/translationalVelocity", translationalVelocity.toString());
    }

    @Override
    public boolean isFinished() {
        xFinished = Math.abs(robotPose.getX() - destinationPose.getX()) < ReefAimConstants.X_TOLERANCE.magnitude();
        yFinished = Math.abs(robotPose.getY() - destinationPose.getY()) < ReefAimConstants.Y_TOLERANCE.magnitude();
        omegaFinished = Swerve.getInstance().aimingReady(0.5);
        SmartDashboard.putBoolean("ReefAimCommand/xFinished", xFinished);
        SmartDashboard.putBoolean("ReefAimCommand/yFinished", yFinished);
        SmartDashboard.putBoolean("ReefAimCommand/omegaFinished", omegaFinished);
        SmartDashboard.putBoolean("ReefAimCommand/emergencyStopped", stop.getAsBoolean());
        return (xFinished && yFinished && omegaFinished) || stop.getAsBoolean();
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