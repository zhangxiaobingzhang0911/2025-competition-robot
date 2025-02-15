package frc.robot.subsystems.intake;

import frc.robot.RobotConstants;
import org.littletonrobotics.junction.AutoLog;

import static frc.robot.RobotConstants.IntakeConstants.IntakePivotGainsClass.*;

public interface IntakePivotIO {
    default void updateInputs(IntakePivotIOInputs inputs) {
    }

    default void setMotorVoltage(double voltage) {
    }

    default void setMotorPosition(double targetPositionDeg) {
    }

    default void resetPosition(double position) {
    }

    @AutoLog
    class IntakePivotIOInputs {
        public double targetPositionDeg = 0.0;
        public double currentPositionDeg = 0.0;
        public double velocityRotPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double statorCurrentAmps = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double tempCelsius = 0.0;

        public double intakePivotKP = INTAKE_PIVOT_KP.get();
        public double intakePivotKI = INTAKE_PIVOT_KI.get();
        public double intakePivotKD = INTAKE_PIVOT_KD.get();
    }
}