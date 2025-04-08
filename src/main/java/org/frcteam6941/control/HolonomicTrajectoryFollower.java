package org.frcteam6941.control;
 
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
 
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
 
import org.littletonrobotics.junction.Logger;
 
// Class responsible for following a holonomic trajectory using PID controllers and feedforward control
public class HolonomicTrajectoryFollower extends PathPlannerTrajectoryFollowerBase<HolonomicDriveSignal> {
    private final PIDController xController;
    private final PIDController yController;
    private final ProfiledPIDController thetaController;
    private final SimpleMotorFeedforward feedforward;
 
    private PathPlannerTrajectoryState lastState = null;
    private Pose2d actualPose = null;
 
    private boolean finished = false;
    private boolean requiredOnTarget = false;
    private boolean lockAngle = true;
 
    private double TARGET_DISTANCE_ACCURACY_REQUIREMENT = 0.2;
    private double TARGET_VELOCITY_ACCURACY_REQUIREMENT = 0.25;
 
    // Constructor initializes the controllers and feedforward model
    public HolonomicTrajectoryFollower(PIDController xController, PIDController yController,
                                       ProfiledPIDController thetaController, SimpleMotorFeedforward feedforward) {
        this.xController = xController;
        this.yController = yController;
        this.thetaController = thetaController;
        this.feedforward = feedforward;
 
        this.xController.setTolerance(TARGET_DISTANCE_ACCURACY_REQUIREMENT, TARGET_VELOCITY_ACCURACY_REQUIREMENT);
        this.yController.setTolerance(TARGET_DISTANCE_ACCURACY_REQUIREMENT, TARGET_VELOCITY_ACCURACY_REQUIREMENT);
    }
 
    // Calculates the drive signal required to follow the trajectory at a given time
    @Override
    protected HolonomicDriveSignal calculateDriveSignal(Pose2d currentPose, Translation2d velocity,
                                                        double rotationalVelocity, PathPlannerTrajectory trajectory, double time,
                                                        double dt) {
        if (time > trajectory.getTotalTimeSeconds()) {
            if (this.requiredOnTarget) {
                if (this.xController.atSetpoint() && this.yController.atSetpoint()) {
                    finished = true;
                    return new HolonomicDriveSignal(new Translation2d(0, 0), 0.0, true, false);
                }
            } else {
                finished = true;
                return new HolonomicDriveSignal(new Translation2d(0, 0), 0.0, true, false);
            }
        }
 
        actualPose = currentPose;
 
        lastState = trajectory.sample(time);
        double x = xController.calculate(currentPose.getX(), lastState.pose.getX());
        double y = yController.calculate(currentPose.getY(), lastState.pose.getY());
        double rotation = 0.0;
        Translation2d translationVector = new Translation2d(x, y);
 
        if (this.lastState != null) {
            Translation2d targetDisplacement = lastState.pose.getTranslation()
                    .minus(this.lastState.pose.getTranslation());
 
            double feedForwardGain = feedforward.getKs() * Math.signum(lastState.linearVelocity)
                    + feedforward.getKv() * lastState.linearVelocity
                    + feedforward.getKa() * lastState.linearVelocity * lastState.timeSeconds;
 
            if (targetDisplacement.getNorm() != 0.00) { // Prevent NaN cases
                Translation2d feedForwardVector = targetDisplacement
                        .times(feedForwardGain / targetDisplacement.getNorm());
                translationVector = translationVector.plus(feedForwardVector);
            }
        }
 
        if (this.lockAngle) {
            rotation = this.thetaController.calculate(currentPose.getRotation().getDegrees(),
                    lastState.pose.getRotation().getDegrees());
        }
 
        return new HolonomicDriveSignal(
                translationVector,
                rotation,
                true,
                false);
    }
 
    // Returns the last sampled state of the trajectory
    public PathPlannerTrajectoryState getLastState() {
        return lastState;
    }
 
    // Sets whether the angle should be locked during path following
    public void setLockAngle(boolean lock) {
        this.lockAngle = lock;
    }
 
    // Sets whether the trajectory follower should require being on target before finishing
    public void setRequiredOnTarget(boolean requiredOnTarget) {
        this.requiredOnTarget = requiredOnTarget;
    }
 
    // Sets the tolerance for the distance and velocity controllers
    public void setTolerance(double distance, double velocity) {
        this.TARGET_DISTANCE_ACCURACY_REQUIREMENT = distance;
        this.TARGET_VELOCITY_ACCURACY_REQUIREMENT = velocity;
    }
 
    // Checks if the trajectory following has finished
    @Override
    protected boolean isFinished() {
        return finished;
    }
 
    // Indicates if the robot is currently following the path
    public boolean isPathFollowing() {
        return !finished;
    }
 
    // Retrieves an array of poses from the current trajectory
    public Pose2d[] getTrajectoryPoses() {
        PathPlannerTrajectory trajectory = getCurrentTrajectory().get();
        Pose2d[] poses = new Pose2d[trajectory.getStates().size()];
        for (int i = 0; i < trajectory.getStates().size(); i++) {
            PathPlannerTrajectoryState state = trajectory.getStates().get(i);
            poses[i] = state.pose;
        }
        return poses;
    }
 
    // Sends data to the logger for debugging or telemetry purposes
    public void sendData() {
        if (isPathFollowing() && this.lastState != null && getCurrentTrajectory() != null) {
            Logger.recordOutput("swerve/PathPlanner/lastState", this.lastState.pose);
            Logger.recordOutput("swerve/PathPlanner/xErrorV", xController.getErrorDerivative());
            Logger.recordOutput("swerve/PathPlanner/xErrorP", xController.getError());
        }
    }
 
    // Resets the controllers and sets the finished flag to false
    @Override
    protected void reset() {
        this.xController.reset();
        this.yController.reset();
        this.finished = false;
    }
}