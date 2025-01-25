package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static edu.wpi.first.units.Units.*;

public class ShooterIOSim implements ShooterIO {
    private static final double LOOP_PERIOD_SECS = 0.02;
    private final DCMotorSim ShooterTalonSim = new DCMotorSim(
            edu.wpi.first.math.system.plant.LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(1),
            0.025, 6.75),
            DCMotor.getKrakenX60(1),
            null
    );


    private Measure<VoltageUnit> ShooterAppliedVoltage = Volts.zero();
    private Measure<AngularVelocityUnit> targetShooterVelocity = RadiansPerSecond.zero();

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        ShooterTalonSim.update(LOOP_PERIOD_SECS);

        inputs.ShooterVelocity =
                RadiansPerSecond.of(ShooterTalonSim.getAngularVelocityRadPerSec());
        inputs.ShooterPosition =
                Radians.of(ShooterTalonSim.getAngularPositionRad());
        inputs.ShooterAppliedVoltage =
                ShooterAppliedVoltage;
        inputs.ShooterSupplyCurrent =
                Amps.of(ShooterTalonSim.getCurrentDrawAmps());

        inputs.targetShooterVelocity = targetShooterVelocity;
    }

    @Override
    public void setFlyWheelDirectVoltage(Measure<VoltageUnit> volts) {
        ShooterAppliedVoltage = volts;
        ShooterTalonSim.setInputVoltage(volts.magnitude());
    }

    @Override
    public void setFlyWheelVelocity(double velocityRPM) {
        ShooterTalonSim.setState(0, velocityRPM);
        targetShooterVelocity = RadiansPerSecond.of(velocityRPM / 60);
    }

    @Override
    public void runVolts(double volts) {

    }

    @Override
    public double getVelocity() {
        return ShooterTalonSim.getAngularVelocityRadPerSec() / 6.28 * 60;
    }

}