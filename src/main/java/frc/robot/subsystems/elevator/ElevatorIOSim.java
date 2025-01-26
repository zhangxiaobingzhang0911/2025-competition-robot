package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.RobotConstants;

import static edu.wpi.first.units.Units.*;

public class ElevatorIOSim implements ElevatorIO {
    private static final double LOOP_PERIOD_SECS = 0.02;
    private final DCMotorSim leftElevatorTalonSim = new DCMotorSim(edu.wpi.first.math.system.plant.LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(1),
    0.025, 6.75), DCMotor.getFalcon500(1), null);
    private final DCMotorSim rightElevatorTalonSim = new DCMotorSim(edu.wpi.first.math.system.plant.LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(1),
    0.025, 6.75), DCMotor.getFalcon500(1) , null);

    private Measure<VoltageUnit> leftElevatorAppliedVoltage = Volts.zero();
    private Measure<VoltageUnit> rightElevatorAppliedVoltage = Volts.zero();
    private Measure<AngularVelocityUnit> targetElevatorVelocity = RadiansPerSecond.zero();

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        leftElevatorTalonSim.update(LOOP_PERIOD_SECS);
        rightElevatorTalonSim.update(LOOP_PERIOD_SECS);

        inputs.leftElevatorVelocity =
                RadiansPerSecond.of(leftElevatorTalonSim.getAngularVelocityRadPerSec());
        inputs.leftElevatorPosition =
                Radians.of(leftElevatorTalonSim.getAngularPositionRad());
        inputs.leftElevatorAppliedVoltage =
                leftElevatorAppliedVoltage;
        inputs.leftElevatorSupplyCurrent =
                Amps.of(leftElevatorTalonSim.getCurrentDrawAmps());

        inputs.rightElevatorVelocity =
                RadiansPerSecond.of(rightElevatorTalonSim.getAngularVelocityRadPerSec());
        inputs.rightElevatorPosition =
                Radians.of(rightElevatorTalonSim.getAngularPositionRad());
        inputs.rightElevatorAppliedVoltage =
                rightElevatorAppliedVoltage;
        inputs.rightElevatorSupplyCurrent =
                Amps.of(rightElevatorTalonSim.getCurrentDrawAmps());

        inputs.targetElevatorVelocity = targetElevatorVelocity;
    }

    @Override
    public void setElevatorDirectVoltage(double volts) {
    }

    public void setElevatorVelocity(double velocityRPM, double ffVoltage) {
        setElevatorVelocity(velocityRPM);
    }

    @Override
    public boolean isNearExtension(double expected) {
        return true;
    }

    @Override
    public void setElevatorVelocity(double velocityRPM) {
        leftElevatorTalonSim.setState(0, velocityRPM);
        targetElevatorVelocity = RadiansPerSecond.of(velocityRPM / 60);
    }

    @Override
    public void setElevatorTarget(double meters) {

    }

    @Override
    public void setElevatorPosition(double position) {
        // Refine this method here for smoother simulation if needed
        double positionDiffInTurns = position - leftElevatorTalonSim.getAngularPositionRotations();
        leftElevatorTalonSim.setState(positionDiffInTurns * 2 * Math.PI, 0);
    }

    @Override
    public double getVelocity() {
        return rightElevatorTalonSim.getAngularVelocityRadPerSec() / 6.28 * 60;
    }

    @Override
    public double getElevatorPosition() {
        return leftElevatorTalonSim.getAngularPositionRotations();
    };

    @Override
    public void brake(){

    }
}
