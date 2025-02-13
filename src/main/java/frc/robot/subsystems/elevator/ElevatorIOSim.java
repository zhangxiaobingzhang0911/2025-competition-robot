package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static edu.wpi.first.units.Units.*;
import static frc.robot.RobotConstants.ElevatorConstants.ELEVATOR_GEAR_RATIO;
import static frc.robot.RobotConstants.ElevatorConstants.ELEVATOR_SPOOL_DIAMETER;

public class ElevatorIOSim implements ElevatorIO {
    private static final double LOOP_PERIOD_SECS = 0.02;
    private final DCMotorSim leaderTalonSim = new DCMotorSim(edu.wpi.first.math.system.plant.LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1),
            0.025, 6.75), DCMotor.getKrakenX60Foc(1), null);
    private Measure<VoltageUnit> leaderAppliedVoltage = Volts.zero();
    private Measure<VoltageUnit> followerAppliedVoltage = Volts.zero();

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        leaderTalonSim.update(LOOP_PERIOD_SECS);

        inputs.appliedVelocity = new double[]{
            (leaderTalonSim.getAngularVelocityRPM())*60,
            leaderTalonSim.getAngularVelocityRPM()*60};
        inputs.appliedPosition = new double[]{
            leaderTalonSim.getAngularPositionRotations(),
            leaderTalonSim.getAngularPositionRotations()};
        inputs.appliedVolts = new double[]{
            leaderAppliedVoltage.magnitude(),
            followerAppliedVoltage.magnitude()};
        inputs.supplyCurrentAmps = new double[]{
            leaderTalonSim.getCurrentDrawAmps(),
            leaderTalonSim.getCurrentDrawAmps()};
            
    }

    @Override
    public void setElevatorDirectVoltage(double volts) {
        leaderAppliedVoltage = Volts.of(volts);
        leaderTalonSim.setInputVoltage(volts);
    }

    @Override
    public void resetElevatorPosition() {
        leaderTalonSim.setAngle(0);;
    }

    @Override
    public boolean isNearExtension(double expected) {
        return MathUtil.isNear(metersToRotations(expected), leaderTalonSim.getAngularPositionRotations(), 0);
    }

    @Override
    public void setElevatorTarget(double meters) {
        // FIXME: Adding delays?
        leaderTalonSim.setAngle(metersToRotations(meters) * 6.28);
    }

    @Override
    public double getElevatorPosition() {
        return leaderTalonSim.getAngularPositionRotations();
    }

    @Override
    public double getElevatorHeight() {
        return rotationsToMeters(leaderTalonSim.getAngularPositionRotations());
    }

    @Override
    public boolean isNearZeroExtension() {
        return MathUtil.isNear(metersToRotations(0.05), leaderTalonSim.getAngularPositionRotations(), 0.3);
    }

    @Override
    public double getElevatorVelocity() {
        return leaderTalonSim.getAngularVelocityRadPerSec() / 6.28 * 60;
    }

    private double metersToRotations(double heightMeters) {
        return (heightMeters / (Math.PI * ELEVATOR_SPOOL_DIAMETER)) * ELEVATOR_GEAR_RATIO;
    }

    private double rotationsToMeters(double rotations) {
        return rotations * (Math.PI * ELEVATOR_SPOOL_DIAMETER) / ELEVATOR_GEAR_RATIO;
    }
}