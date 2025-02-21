package frc.robot.auto.basics;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventScheduler;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;
import org.littletonrobotics.AllianceFlipUtil;

import static frc.robot.RobotConstants.SwerveConstants.AUTO_SWERVE_TOLERANCE_SECS;

// adapted from FollowPathCommand in pathplanner
public class FollowPath extends Command {
    private final Swerve swerve;
    private final Timer timer = new Timer();
    private final EventScheduler eventScheduler;
    private final boolean angleLock;
    private final boolean requiredOnTarget;
    private final boolean resetOdometry;
    private PathPlannerPath path;
    private PathPlannerTrajectory trajectory;
    private FollowTrajectory followTrajectoryCommand;

    public FollowPath(Swerve swerve, PathPlannerPath path, boolean angleLock, boolean requiredOnTarget, boolean resetOdometry) {
        this.swerve = swerve;
        this.eventScheduler = new EventScheduler();
        this.path = path;
        this.angleLock = angleLock;
        this.requiredOnTarget = requiredOnTarget;
        this.resetOdometry = resetOdometry;
    }

    private void initializeTrajectory() {
        Pose2d currentPose = this.swerve.getLocalizer().getCoarseFieldPose(0);
        ChassisSpeeds currentSpeeds = this.swerve.getChassisSpeeds();
        double linearVel = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
        if (this.path.getIdealStartingState() != null) {
            boolean idealVelocity = Math.abs(linearVel - this.path.getIdealStartingState().velocityMPS()) <= (double) 0.25F;
            boolean idealRotation = !this.swerve.getAutoConfig().isHolonomic ||
                    Math.abs(currentPose.getRotation().minus(this.path.getIdealStartingState().rotation()).getDegrees()) <= (double) 30.0F;
            if (idealVelocity && idealRotation) {
                this.trajectory = this.path.getIdealTrajectory(this.swerve.getAutoConfig()).orElseThrow();
            } else {
                this.trajectory = this.path.generateTrajectory(currentSpeeds, currentPose.getRotation(), this.swerve.getAutoConfig());
            }
        } else {
            this.trajectory = this.path.generateTrajectory(currentSpeeds, currentPose.getRotation(), this.swerve.getAutoConfig());
        }
    }

    @Override
    public void initialize() {
        if (AllianceFlipUtil.shouldFlip() && !this.path.preventFlipping) {
            this.path = this.path.flipPath();
        }

        this.trajectory = this.path.getIdealTrajectory(this.swerve.getAutoConfig()).orElseThrow();

        if (resetOdometry) {
            swerve.resetPose(trajectory.getInitialPose());
        }

        initializeTrajectory();

        followTrajectoryCommand = new FollowTrajectory(this.swerve, this.trajectory, this.angleLock, this.requiredOnTarget);
        followTrajectoryCommand.initialize();

        PathPlannerAuto.setCurrentTrajectory(this.trajectory);
        PathPlannerAuto.currentPathName = this.path.name;
        PathPlannerLogging.logActivePath(this.path);
        PPLibTelemetry.setCurrentPath(this.path);
        this.eventScheduler.initialize(this.trajectory);
        this.timer.reset();
        this.timer.start();
    }

    @Override
    public void execute() {
        double currentTime = this.timer.get();
        this.eventScheduler.execute(currentTime);
    }

    @Override
    public boolean isFinished() {
        return followTrajectoryCommand.isFinished() && this.timer.get() >= AUTO_SWERVE_TOLERANCE_SECS;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopMovement();
        this.timer.stop();
        this.eventScheduler.end();
    }
}
