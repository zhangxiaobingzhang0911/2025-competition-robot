package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    void updateInputs(ElevatorIOInputs inputs);

    void setElevatorVoltage(double volts);

    void setElevatorTarget(double meters);

    void resetElevatorPosition();

    double getElevatorVelocity();

    double getElevatorHeight();

    boolean isNearExtension(double expected);

    boolean isNearZeroExtension();

    @AutoLog
    class ElevatorIOInputs {
        public double positionMeters = 0.0;
        public double velocityMetersPerSec = 0.0;
        public double setpointMeters = 0.0;
        public double appliedVolts = 0.0;
        public double statorCurrentAmps = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
    }

}