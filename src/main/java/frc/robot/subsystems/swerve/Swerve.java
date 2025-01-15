package frc.robot.subsystems.swerve;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.team254.lib.util.MovingAverage;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.SwerveConstants;
import frc.robot.utils.AllianceFlipUtil;
import lombok.Getter;
import lombok.Setter;
import lombok.Synchronized;
import org.frcteam6941.control.HolonomicDriveSignal;
import org.frcteam6941.control.HolonomicTrajectoryFollower;
import org.frcteam6941.drivers.DummyGyro;
import org.frcteam6941.drivers.Gyro;
import org.frcteam6941.drivers.Pigeon2Gyro;
import org.frcteam6941.localization.Localizer;
import org.frcteam6941.localization.SwerveDeltaCoarseLocalizer;
import org.frcteam6941.looper.Updatable;
import org.frcteam6941.swerve.*;
import org.frcteam6941.utils.AngleNormalization;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public class Swerve implements Updatable, Subsystem {
    private static Swerve instance;
    private final SwerveModuleBase[] swerveMods;
    private final SwerveDriveKinematics swerveKinematics;
    private final SwerveDeltaCoarseLocalizer swerveLocalizer;
    @Getter
    private final Gyro gyro;
    private final SwerveSetpointGenerator generator;
    // System Status
    private final MovingAverage pitchVelocity;
    private final MovingAverage rollVelocity;
    private final MovingAverage yawVelocity;
    // Snap Rotation Controller
    private final ProfiledPIDController headingController = new ProfiledPIDController(
            RobotConstants.SwerveConstants.headingController.HEADING_KP.get(),
            RobotConstants.SwerveConstants.headingController.HEADING_KI.get(),
            RobotConstants.SwerveConstants.headingController.HEADING_KD.get(),
            new TrapezoidProfile.Constraints(400, 720));
    // Path Following Controller
    private final HolonomicTrajectoryFollower trajectoryFollower = new HolonomicTrajectoryFollower(
            new PIDController(3.5, 0.0, 0.0), new PIDController(3.5, 0.0, 0.0),
            this.headingController, RobotConstants.SwerveConstants.DRIVETRAIN_FEEDFORWARD);
    private boolean isLockHeading;
    /**
     * -- GETTER --
     * Get the lock heading target for the swerve drive.
     */
    @Getter
    private double headingTarget = 0.0;
    @Getter
    @Setter
    private Double overrideRotation = null;
    @Getter
    @Setter
    private double headingVelocityFeedforward = 0.00;
    // Control Targets
    private HolonomicDriveSignal driveSignal = new HolonomicDriveSignal(new edu.wpi.first.math.geometry.Translation2d(), 0.0, true, false);
    private SwerveSetpoint setpoint;
    private SwerveSetpoint previousSetpoint;
    @Getter
    private org.frcteam6941.swerve.SwerveSetpointGenerator.KinematicLimits kinematicLimits;
    @Getter
    @Setter
    private State state = State.DRIVE;

    private Swerve() {
        if (RobotBase.isReal()) {
            swerveMods = new SwerveModuleBase[]{
                    new CTRESwerveModule(0, RobotConstants.SwerveConstants.FrontLeft,
                            RobotConstants.CAN_BUS_NAME),
                    new CTRESwerveModule(1, RobotConstants.SwerveConstants.FrontRight,
                            RobotConstants.CAN_BUS_NAME),
                    new CTRESwerveModule(2, RobotConstants.SwerveConstants.BackLeft,
                            RobotConstants.CAN_BUS_NAME),
                    new CTRESwerveModule(3, RobotConstants.SwerveConstants.BackRight,
                            RobotConstants.CAN_BUS_NAME),
            };
            gyro = new Pigeon2Gyro(RobotConstants.SwerveConstants.PIGEON_ID, RobotConstants.CAN_BUS_NAME);
        } else {
            swerveMods = new SwerveModuleBase[]{
                    new SimSwerveModuleDummy(0, RobotConstants.SwerveConstants.FrontLeft),
                    new SimSwerveModuleDummy(1, RobotConstants.SwerveConstants.FrontRight),
                    new SimSwerveModuleDummy(2, RobotConstants.SwerveConstants.BackLeft),
                    new SimSwerveModuleDummy(3, RobotConstants.SwerveConstants.BackRight),
            };
            gyro = new DummyGyro(RobotConstants.LOOPER_DT);

        }
        headingController.setIntegratorRange(-0.5, 0.5);
        headingController.enableContinuousInput(0, 360.0);
        swerveKinematics = new SwerveDriveKinematics(
                RobotConstants.SwerveConstants.modulePlacements);
        swerveLocalizer = new SwerveDeltaCoarseLocalizer(swerveKinematics, 50, 20, 20, getModulePositions());

        gyro.setYaw(0.0);
        swerveLocalizer.reset(new Pose2d(), getModulePositions());

        yawVelocity = new MovingAverage(10);
        pitchVelocity = new MovingAverage(10);
        rollVelocity = new MovingAverage(10);

        setpoint = new SwerveSetpoint(new edu.wpi.first.math.kinematics.ChassisSpeeds(), getModuleStates());
        previousSetpoint = new SwerveSetpoint(new edu.wpi.first.math.kinematics.ChassisSpeeds(), getModuleStates());
        generator = new SwerveSetpointGenerator(RobotConstants.SwerveConstants.modulePlacements);
        kinematicLimits = RobotConstants.SwerveConstants.DRIVETRAIN_UNCAPPED;


    }

    private static double getDriveBaseRadius() {
        var moduleLocations = new edu.wpi.first.math.geometry.Translation2d[]{
                new edu.wpi.first.math.geometry.Translation2d(RobotConstants.SwerveConstants.FrontLeft.LocationX,
                        RobotConstants.SwerveConstants.FrontLeft.LocationY),
                new edu.wpi.first.math.geometry.Translation2d(RobotConstants.SwerveConstants.FrontRight.LocationX,
                        RobotConstants.SwerveConstants.FrontRight.LocationY),
                new edu.wpi.first.math.geometry.Translation2d(RobotConstants.SwerveConstants.BackLeft.LocationX,
                        RobotConstants.SwerveConstants.BackLeft.LocationY),
                new edu.wpi.first.math.geometry.Translation2d(RobotConstants.SwerveConstants.BackRight.LocationX,
                        RobotConstants.SwerveConstants.BackRight.LocationY)
        };

        var driveBaseRadius = 0.0;
        for (var moduleLocation : moduleLocations)
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        return driveBaseRadius;
    }

    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }


    public edu.wpi.first.math.kinematics.ChassisSpeeds getChassisSpeeds() {
        return swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Core methods to update the odometry of swerve based on module states.
     *
     * @param time Current time stamp.
     * @param dt   Delta time between updates.
     */
    private void updateOdometry(double time, double dt) {
        swerveLocalizer.updateWithTime(time, dt, gyro.getYaw(), getModulePositions());
    }

    /**
     * Core method to update swerve modules according to the
     * {@link HolonomicDriveSignal} given.
     *
     * @param driveSignal The holonomic drive signal.
     * @param dt          Delta time between updates.
     */
    private void updateModules(HolonomicDriveSignal driveSignal, double dt) {
        edu.wpi.first.math.kinematics.ChassisSpeeds desiredChassisSpeed;

        if (driveSignal == null) {
            desiredChassisSpeed = new edu.wpi.first.math.kinematics.ChassisSpeeds(0.0, 0.0, 0.0);
            driveSignal = new HolonomicDriveSignal(new edu.wpi.first.math.geometry.Translation2d(), 0.0, true, false);
        } else {

            double x = driveSignal.getTranslation().getX();
            double y = driveSignal.getTranslation().getY();
            double rotation = driveSignal.getRotation();

            Rotation2d robotAngle = swerveLocalizer.getLatestPose().getRotation();

            //flip drive signal for red side (no need to flip auto)
            if (driveSignal.isFieldOriented())
                if (AllianceFlipUtil.shouldFlip() && this.state != State.PATH_FOLLOWING) {
                    desiredChassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rotation,
                            robotAngle.rotateBy(Rotation2d.fromDegrees(180)));
                } else {
                    desiredChassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rotation,
                            robotAngle.rotateBy(Rotation2d.fromDegrees(0)));
                }

            else
                desiredChassisSpeed = new edu.wpi.first.math.kinematics.ChassisSpeeds(x, y, rotation);
        }

        edu.wpi.first.math.geometry.Twist2d twist = new Pose2d().log(new Pose2d(
                new edu.wpi.first.math.geometry.Translation2d(
                        desiredChassisSpeed.vxMetersPerSecond * RobotConstants.LOOPER_DT,
                        desiredChassisSpeed.vyMetersPerSecond * RobotConstants.LOOPER_DT),
                new Rotation2d(
                        desiredChassisSpeed.omegaRadiansPerSecond * RobotConstants.LOOPER_DT)));

        desiredChassisSpeed = new edu.wpi.first.math.kinematics.ChassisSpeeds(
                twist.dx / RobotConstants.LOOPER_DT,
                twist.dy / RobotConstants.LOOPER_DT,
                twist.dtheta / RobotConstants.LOOPER_DT);

        setpoint = generator.generateSetpoint(
                kinematicLimits, previousSetpoint, desiredChassisSpeed, dt);
        previousSetpoint = setpoint;
        Logger.recordOutput("swerve/Kinematics/DesiredSpeedy", desiredChassisSpeed.vyMetersPerSecond);

        for (SwerveModuleBase mod : swerveMods) {
            mod.setDesiredState(setpoint.mModuleStates[mod.getModuleNumber()], driveSignal.isOpenLoop(), false);
        }
    }

    public com.team254.lib.geometry.Twist2d getChassisTwist() {
        edu.wpi.first.math.kinematics.ChassisSpeeds speeds = getChassisSpeeds();
        return new com.team254.lib.geometry.Twist2d(
                speeds.vxMetersPerSecond * RobotConstants.LOOPER_DT,
                speeds.vyMetersPerSecond * RobotConstants.LOOPER_DT,
                speeds.omegaRadiansPerSecond * RobotConstants.LOOPER_DT);
    }

    @Synchronized
    public double getYawVelocity() {
        return yawVelocity.getAverage();
    }

    @Synchronized
    public double getPitchVelocity() {
        return pitchVelocity.getAverage();
    }

    @Synchronized
    public double getRollVelocity() {
        return rollVelocity.getAverage();
    }

    /**
     * Core method to drive the swerve drive. Note that any trajectory following
     * signal will be canceled when this method is called.
     *
     * @param translationalVelocity Normalized translation vector of the swerve
     *                              drive.
     * @param rotationalVelocity    Normalized rotational magnitude of the swerve
     *                              drive.
     * @param isFieldOriented       Is the drive signal field oriented.
     */
    public void drive(edu.wpi.first.math.geometry.Translation2d translationalVelocity, double rotationalVelocity,
                      boolean isFieldOriented, boolean isOpenLoop) {

        if (Math.abs(translationalVelocity.getX()) < RobotConstants.SwerveConstants.deadband) {
            translationalVelocity = new edu.wpi.first.math.geometry.Translation2d(0, translationalVelocity.getY());
        }
        if (Math.abs(translationalVelocity.getY()) < RobotConstants.SwerveConstants.deadband) {
            translationalVelocity = new edu.wpi.first.math.geometry.Translation2d(translationalVelocity.getX(), 0);
        }
        if (Math.abs(rotationalVelocity) < RobotConstants.SwerveConstants.rotationalDeadband) {
            rotationalVelocity = 0;
        }
        driveSignal = new HolonomicDriveSignal(translationalVelocity, rotationalVelocity, isFieldOriented, isOpenLoop);
    }

    public void follow(PathPlannerTrajectory targetTrajectory, boolean isLockAngle,
                       boolean requiredOnTarget) {
        this.trajectoryFollower.setLockAngle(isLockAngle);
        this.trajectoryFollower.setRequiredOnTarget(requiredOnTarget);
        resetHeadingController();
        if (targetTrajectory == null) {
            this.setState(State.PATH_FOLLOWING);
            trajectoryFollower.cancel();
        }
        this.trajectoryFollower.follow(targetTrajectory);
        Logger.recordOutput("swerve/PathPlanner/trajectory", this.trajectoryFollower.getTrajectoryPoses());
    }

    public void cancelFollow() {
        this.trajectoryFollower.cancel();
    }

    public HolonomicTrajectoryFollower getFollower() {
        return this.trajectoryFollower;
    }

    public void pointWheelsAt(Rotation2d rotation2d) {
        for (SwerveModuleBase mod : swerveMods) {
            setpoint.mModuleStates[mod.getModuleNumber()].angle = rotation2d;
            setpoint.mModuleStates[mod.getModuleNumber()].speedMetersPerSecond = 0.0;
            mod.setDesiredState(setpoint.mModuleStates[mod.getModuleNumber()], true, false);
        }
    }

    public void brake() {
        setState(State.BRAKE);
    }

    public void normal() {
        setState(State.DRIVE);
    }

    public void empty() {
        setState(State.EMPTY);
    }

    public void auto() {
        setState(State.PATH_FOLLOWING);
    }

    public void stopMovement() {
        driveSignal = new HolonomicDriveSignal(new edu.wpi.first.math.geometry.Translation2d(), 0.0, true, false);
    }

    public void setKinematicsLimit(org.frcteam6941.swerve.SwerveSetpointGenerator.KinematicLimits limit) {
        kinematicLimits = limit;
    }

    public void resetPose(Pose2d resetPose) {
        gyro.setYaw(resetPose.getRotation().getDegrees());
        swerveLocalizer.reset(resetPose, getModulePositions());
    }

    /**
     * Set the state of the module independently.
     *
     * @param desiredStates The states of the model.
     * @param isOpenLoop    Whether to use open loop control
     */
    public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop, boolean overrideMotion) {
        if (isOpenLoop) {
            SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
                    RobotConstants.SwerveConstants.MAX_VOLTAGE.magnitude());
        }

        for (SwerveModuleBase mod : swerveMods) {
            mod.setDesiredState(desiredStates[mod.getModuleNumber()], isOpenLoop, overrideMotion);
        }
    }

    /**
     * Convenience method to set the wheels in X shape to resist impacts.
     */
    private void setModuleStatesBrake() {
        for (SwerveModuleBase mod : swerveMods) {
            edu.wpi.first.math.geometry.Translation2d modulePosition = RobotConstants.SwerveConstants.modulePlacements[mod.getModuleNumber()];
            Rotation2d angle = new Rotation2d(modulePosition.getX(), modulePosition.getY());
            mod.setDesiredState(new SwerveModuleState(0.0, angle.plus(Rotation2d.fromDegrees(180.0))), false, true);
        }
    }

    public void resetYaw(double degree) {
        this.gyro.setYaw(degree);
    }

    public void resetRoll(double degree) {
        this.gyro.setRoll(degree);
    }

    public void resetPitch(double degree) {
        this.gyro.setPitch(degree);
    }

    public Localizer getLocalizer() {
        return this.swerveLocalizer;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[swerveMods.length];
        for (SwerveModuleBase mod : swerveMods) {
            states[mod.getModuleNumber()] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[swerveMods.length];
        for (SwerveModuleBase mod : swerveMods) {
            positions[mod.getModuleNumber()] = mod.getPosition();
        }
        return positions;
    }

    public void setHeadingControllerPID() {
        headingController.setPID(
                RobotConstants.SwerveConstants.headingController.HEADING_KP.get(),
                RobotConstants.SwerveConstants.headingController.HEADING_KI.get(),
                RobotConstants.SwerveConstants.headingController.HEADING_KD.get());
    }

    /*
     * Reset heading controller according to current drivetrain status.
     */
    public void resetHeadingController() {
        headingController.reset(
                swerveLocalizer.getLatestPose().getRotation().getDegrees(),
                getYawVelocity());
    }

    /**
     * Return if the swerve drive has a set heading target.
     *
     * @return If swerve is in lock heading.
     */
    public boolean isLockHeading() {
        return this.isLockHeading;
    }

    /**
     * Set if swerve will enter lock heading.
     *
     * @param status Boolean value for enabling or disabling lock heading.
     */
    public void setLockHeading(boolean status) {
        if (this.isLockHeading != status) {
            headingController.reset(gyro.getYaw().getDegrees(), getYawVelocity());
        }
        this.isLockHeading = status;
        this.headingVelocityFeedforward = 0.0;
    }

    public synchronized void setHeadingTarget(double heading) {
        double target = heading;
        double position = gyro.getYaw().getDegrees();

        while (position - target > 180) {
            target += 360;
        }

        while (target - position > 180) {
            target -= 360;
        }

        headingTarget = target;
    }

    public boolean isHeadingOnTarget() {
        return this.headingController.atSetpoint();
    }

    public void clearOverrideRotation() {
        overrideRotation = null;
    }

    @Override
    public void read(double time, double dt) {
        for (SwerveModuleBase mod : swerveMods) {
            mod.updateSignals();
        }
        updateOdometry(time, dt);
    }

    @Override
    public void update(double time, double dt) {
        Optional<HolonomicDriveSignal> trajectorySignal = trajectoryFollower.update(
                swerveLocalizer.getCoarseFieldPose(time),
                swerveLocalizer.getMeasuredVelocity().getTranslation(),
                swerveLocalizer.getMeasuredVelocity().getRotation().getDegrees(),
                time, dt);
        if (trajectorySignal.isPresent()) {
            driveSignal = trajectorySignal.get();
        } else if (isLockHeading) {
            headingTarget = AngleNormalization.placeInAppropriate0To360Scope(gyro.getYaw().getDegrees(), headingTarget);

            // clamp max rotation output value from the heading controller to prevent controller overshoot
            double headingRotationLimit = SwerveConstants.headingController.MAX_ERROR_CORRECTION_ANGLE.get()
                    * SwerveConstants.headingController.HEADING_KP.get();
            double rotation = MathUtil
                    .clamp(headingController.calculate(gyro.getYaw().getDegrees(), new TrapezoidProfile.State(
                            headingTarget, headingVelocityFeedforward)), -headingRotationLimit, headingRotationLimit);

            driveSignal = new HolonomicDriveSignal(driveSignal.getTranslation(), rotation,
                    driveSignal.isFieldOriented(), driveSignal.isOpenLoop());
            //
            //            Logger.recordOutput("swerve/heading/rotation", rotation);
            //            Logger.recordOutput("swerve/heading/gyro", gyro.getYaw().getDegrees());
            //            Logger.recordOutput("swerve/heading/target", headingTarget);
            //            Logger.recordOutput("swerve/heading/difference", Math.abs(headingTarget - gyro.getYaw().getDegrees()));

        } else if (overrideRotation != null) {
            driveSignal = new HolonomicDriveSignal(driveSignal.getTranslation(), overrideRotation,
                    driveSignal.isFieldOriented(), driveSignal.isOpenLoop());
        }

        rollVelocity.addNumber(gyro.getRaw()[0]);
        pitchVelocity.addNumber(gyro.getRaw()[1]);
        yawVelocity.addNumber(gyro.getRaw()[2]);
    }

    @Override
    public void write(double time, double dt) {
        switch (state) {
            case BRAKE:
                setModuleStatesBrake();
                break;
            case DRIVE:
            case PATH_FOLLOWING:
                updateModules(driveSignal, dt);
                break;
            case EMPTY:
                break;
        }
    }

    @Override
    public void telemetry() {
        if (RobotConstants.TUNING) {
            setHeadingControllerPID();
            SmartDashboard.putString("swerve/localizer/latest_pose", getLocalizer().getLatestPose().toString());
            SmartDashboard.putString("swerve/localizer/accel", getLocalizer().getMeasuredAcceleration().toString());
            SmartDashboard.putString("swerve/localizer/velocity", getLocalizer().getSmoothedVelocity().toString());
        }
        Logger.recordOutput("swerve/localizer/CoarsedFieldPose", getLocalizer().getCoarseFieldPose(0));
        Logger.recordOutput("swerve/localizer/LatestPose", getLocalizer().getLatestPose());
        Logger.recordOutput("swerve/isLockHeading", isLockHeading);
        Logger.recordOutput("swerve/DriveSignalRotation", driveSignal.getRotation());
        Logger.recordOutput("swerve/PathPlanner/IsPathFollowing", trajectoryFollower.isPathFollowing());
        Logger.recordOutput("swerve/localizer/GyroAngle", gyro.getYaw());
        Logger.recordOutput("swerve/localizer/MeasuredVelocity", swerveLocalizer.getMeasuredVelocity());
        Logger.recordOutput("swerve/localizer/MeasuredAcceleration", swerveLocalizer.getMeasuredAcceleration());

        trajectoryFollower.sendData();
        //Logger.recordOutput("ActivePath", PathPlannerPath.fromPathFile("T_1").getPathPoses());


    }

    @Override
    public void stop() {
        stopMovement();
        setState(State.DRIVE);
    }

    @Override
    public void simulate(double time, double dt) {
        gyro.setYaw(
                gyro.getYaw().rotateBy(
                        new Rotation2d(dt * setpoint.mChassisSpeeds.omegaRadiansPerSecond)).getDegrees());
        read(time, dt);
    }

    public boolean aimingReady(double offset) {
        var dtReady = Math.abs(gyro.getYaw().getDegrees() - headingTarget) < offset;
        SmartDashboard.putBoolean("SwerveReady", dtReady);
        boolean angularSpeedReady = this.getLocalizer().getSmoothedVelocity().getRotation().getDegrees() < 2.14;
        SmartDashboard.putBoolean("SwerveAngularReady", angularSpeedReady);
        return dtReady && angularSpeedReady;
    }

    public enum State {
        BRAKE, DRIVE, PATH_FOLLOWING, EMPTY
    }
}
    