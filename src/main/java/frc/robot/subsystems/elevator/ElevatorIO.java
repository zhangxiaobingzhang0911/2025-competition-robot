package frc.robot.subsystems.elevator;

import edu.wpi.first.units.*;
import frc.robot.RobotConstants;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface ElevatorIO {
    void updateInputs(ElevatorIOInputs inputs);

    void setElevatorDirectVoltage(double volts);
    void setElevatorTarget(double meters);
    void resetElevatorPosition();
    void zeroingElevator();
    double getElevatorVelocity();
    double getElevatorPosition();
    boolean isNearExtension(double expected);
    boolean isCurrentMax();

    @AutoLog
    class ElevatorIOInputs {
        public Measure<AngularVelocityUnit> leftElevatorVelocity = RadiansPerSecond.zero();
        public Measure<AngleUnit> leftElevatorPosition = Radians.zero();
        public Measure<VoltageUnit> leftElevatorAppliedVoltage = Volts.zero();
        public Measure<CurrentUnit> leftElevatorSupplyCurrent = Amps.zero();

        public Measure<AngularVelocityUnit> rightElevatorVelocity = RadiansPerSecond.zero();
        public Measure<AngleUnit> rightElevatorPosition = Radians.zero();
        public Measure<VoltageUnit> rightElevatorAppliedVoltage = Volts.zero();
        public Measure<CurrentUnit> rightElevatorSupplyCurrent = Amps.zero();

        public Measure<AngularVelocityUnit> targetElevatorVelocity = RadiansPerSecond.zero();

        public double[] statorCurrentAmps = new double[] {};

        public double ElevatorKP = frc.robot.RobotConstants.ElevatorGainsClass.ELEVATOR_KP.get();
        public double ElevatorKI = frc.robot.RobotConstants.ElevatorGainsClass.ELEVATOR_KI.get();
        public double ElevatorKD = frc.robot.RobotConstants.ElevatorGainsClass.ELEVATOR_KD.get();
        public double ElevatorKA = frc.robot.RobotConstants.ElevatorGainsClass.ELEVATOR_KA.get();
        public double ElevatorKV = frc.robot.RobotConstants.ElevatorGainsClass.ELEVATOR_KV.get();
        public double ElevatorKS = frc.robot.RobotConstants.ElevatorGainsClass.ELEVATOR_KS.get();
        public double ElevatorKG = frc.robot.RobotConstants.ElevatorGainsClass.ELEVATOR_KG.get();
    }
}