package frc.robot.subsystems.elevator;

import edu.wpi.first.units.*;
import frc.robot.RobotConstants.ElevatorConstants.ElevatorGainsClass;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs {
        public double positionMeters = 0.0;
        public double velocityMetersPerSec = 0.0;
        public double motionMagicVelocityTarget = 0.0;
        public double motionMagicPositionTarget = 0.0;
        public double setpointMeters = 0.0;
        public double[] appliedPosition = new double[] {}; // {leader, follower}
        public double[] appliedVelocity = new double[] {}; // {leader, follower}
        public double[] appliedVolts = new double[] {}; // {leader, follower}
        public double[] statorCurrentAmps = new double[] {}; // {leader, follower}
        public double[] supplyCurrentAmps = new double[] {}; // {leader, follower}
        public double[] tempCelsius = new double[] {}; // {leader, follower}
        // public double acceleration = 0.0; may add later
    }

    void updateInputs(ElevatorIOInputs inputs);

    void setElevatorDirectVoltage(double volts);

    void setElevatorTarget(double meters);

    void resetElevatorPosition();

    double getElevatorVelocity();

    double getElevatorPosition();

    double getElevatorHeight();

    boolean isNearExtension(double expected);

    boolean isNearZeroExtension();
}