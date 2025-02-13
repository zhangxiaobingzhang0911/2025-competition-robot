package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.RobotConstants.ElevatorGainsClass;

import static edu.wpi.first.units.Units.*;
import static frc.robot.RobotConstants.ElevatorConstants.ELEVATOR_GEAR_RATIO;
import static frc.robot.RobotConstants.ElevatorConstants.ELEVATOR_SPOOL_DIAMETER;
import static frc.robot.RobotConstants.ElevatorConstants.MAX_EXTENSION_METERS;

public class ElevatorIOSim implements ElevatorIO {
    private static final double LOOP_PERIOD_SECS = 0.02;
    private final ElevatorSim physicsSim =
    new ElevatorSim(
            DCMotor.getKrakenX60Foc(2),
            6.75,
            // Add half of first stage mass bc its on a 2:1 ratio compared to carriage
            Units.lbsToKilograms(7.0 + (3.25 / 2)),
            ELEVATOR_SPOOL_DIAMETER/2,
            0.0,
            MAX_EXTENSION_METERS.get(),
            true,
            0.0);
    private double volts = 0.0;
    private final ProfiledPIDController pid =
        new ProfiledPIDController(ElevatorGainsClass.ELEVATOR_KP.get(),ElevatorGainsClass.ELEVATOR_KI.get(),ElevatorGainsClass.ELEVATOR_KD.get(),
        new Constraints(5.0, 10.0));
    private final ElevatorFeedforward ff =
        new ElevatorFeedforward(
            ElevatorGainsClass.ELEVATOR_KS.get(),
            ElevatorGainsClass.ELEVATOR_KG.get(),
            ElevatorGainsClass.ELEVATOR_KV.get(),
            (DCMotor.getKrakenX60Foc(1).KvRadPerSecPerVolt * ELEVATOR_SPOOL_DIAMETER/2)
            / ELEVATOR_GEAR_RATIO);
    private Measure<VoltageUnit> leaderAppliedVoltage = Volts.zero();
    private Measure<VoltageUnit> followerAppliedVoltage = Volts.zero();

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        physicsSim.update(LOOP_PERIOD_SECS);

        inputs.appliedVelocity = new double[]{
            (physicsSim.getVelocityMetersPerSecond()/ (Math.PI * ELEVATOR_SPOOL_DIAMETER)) * ELEVATOR_GEAR_RATIO,
            (physicsSim.getVelocityMetersPerSecond()/ (Math.PI * ELEVATOR_SPOOL_DIAMETER)) * ELEVATOR_GEAR_RATIO,};
        inputs.appliedPosition = new double[]{
            (physicsSim.getPositionMeters()/ (Math.PI * ELEVATOR_SPOOL_DIAMETER)) * ELEVATOR_GEAR_RATIO,
            (physicsSim.getPositionMeters()/ (Math.PI * ELEVATOR_SPOOL_DIAMETER)) * ELEVATOR_GEAR_RATIO,};
        inputs.appliedVolts = new double[]{
            leaderAppliedVoltage.magnitude(),
            followerAppliedVoltage.magnitude()};
        inputs.supplyCurrentAmps = new double[]{
            physicsSim.getCurrentDrawAmps(),
            physicsSim.getCurrentDrawAmps()};
        inputs.statorCurrentAmps = new double[]{
                physicsSim.getCurrentDrawAmps(),
                physicsSim.getCurrentDrawAmps()
        };
            
    }

    @Override
    public void setElevatorDirectVoltage(double volts) {
        volts = leaderAppliedVoltage.magnitude();
        physicsSim.setInputVoltage(MathUtil.clamp(volts, -12.0, 12.0));
    }

    @Override
    public void resetElevatorPosition() {
        // physicsSim.setState(0,0);
    }

    @Override
    public boolean isNearExtension(double expected) {
        return true;
    }

    @Override
    public void setElevatorTarget(double meters) {
        // FIXME: Adding delays?
        setElevatorDirectVoltage(
        pid.calculate(physicsSim.getPositionMeters(), meters)
            + ff.calculate(pid.getSetpoint().velocity));
    }

    @Override
    public double getElevatorPosition() {
        return metersToRotations(physicsSim.getPositionMeters());
    }

    @Override
    public double getElevatorHeight() {
        return rotationsToMeters(physicsSim.getPositionMeters());
    }

    @Override
    public boolean isNearZeroExtension() {
        return MathUtil.isNear(metersToRotations(0.05), metersToRotations(physicsSim.getPositionMeters()), 0.3);
    }

    @Override
    public double getElevatorVelocity() {
        return metersToRotations(physicsSim.getVelocityMetersPerSecond());
    }

    private double metersToRotations(double heightMeters) {
        return (heightMeters / (Math.PI * ELEVATOR_SPOOL_DIAMETER)) * ELEVATOR_GEAR_RATIO;
    }

    private double rotationsToMeters(double rotations) {
        return rotations * (Math.PI * ELEVATOR_SPOOL_DIAMETER) / ELEVATOR_GEAR_RATIO;
    }
}