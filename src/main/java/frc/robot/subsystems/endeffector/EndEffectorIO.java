package frc.robot.subsystems.endeffector;

import org.littletonrobotics.junction.AutoLog;

import static frc.robot.RobotConstants.EndEffectorConstants.EndEffectorGainsClass.*;

public interface EndEffectorIO {
    @AutoLog
    class EndEffectorIOInputs {
        public double velocityRotationsPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double tempCelsius = 0.0;

        public boolean isHigherEndEffectorBeamBreakOn = false;
        public boolean islowerEndEffectorBeamBreakOn = false;

        public double endEffectorKP = ENDEFFECTOR_KP.get();
        public double endEffectorKI = ENDEFFECTOR_KI.get();
        public double endEffectorKD = ENDEFFECTOR_KD.get();
        public double endEffectorKA = ENDEFFECTOR_KA.get();
        public double endEffectorKV = ENDEFFECTOR_KV.get();
        public double endEffectorKS = ENDEFFECTOR_KS.get();
    }

    /** Update the set of loggable inputs. */
    public default void updateInputs(EndEffectorIOInputs inputs) {}

    /** Run open loop at the specified voltage. */
    public default void setVoltage(double voltage) {}

    /** Run close loop at the specified velocity. */
    public default void setVelocity(double velocityRPS) {}
}


