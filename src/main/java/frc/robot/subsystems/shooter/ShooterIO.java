package frc.robot.subsystems.shooter;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;
import static frc.robot.RobotConstants.ShooterConstants.shooterGainsClass.*;
import static frc.robot.RobotConstants.ShooterConstants.shooterGainsClass.SHOOTER_KP;

public interface ShooterIO {
    void updateInputs(ShooterIOInputs inputs);

    void setFlyWheelDirectVoltage(Measure<VoltageUnit> volts);

    void setFlyWheelVelocity(double velocityRPM);

    void runVolts(double volts);

    double getVelocity();


    @AutoLog
    class ShooterIOInputs {
        public Measure<AngularVelocityUnit> ShooterVelocity = RadiansPerSecond.zero();
        public Measure<AngleUnit> ShooterPosition = Radians.zero();
        public Measure<VoltageUnit> ShooterAppliedVoltage = Volts.zero();
        public Measure<CurrentUnit> ShooterSupplyCurrent = Amps.zero();

        public Measure<AngularVelocityUnit> targetShooterVelocity = RadiansPerSecond.zero();

        public double ShooterKP = SHOOTER_KP.get();
        public double ShooterKI = SHOOTER_KI.get();
        public double ShooterKD = SHOOTER_KD.get();
        public double ShooterKA = SHOOTER_KA.get();
        public double ShooterKV = SHOOTER_KV.get();
        public double ShooterKS = SHOOTER_KS.get();
    }
}

