package frc.robot.subsystems.intaker;

import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.AngularVelocity;

import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface IntakerIO {
    default void updateInputs(IntakerIOInputs inputs) {
    }

    default void setVelocity(double velocity) {
    }

    default void setVoltage(Measure<VoltageUnit> voltage) {
    }


    @AutoLog
    class IntakerIOInputs {
        public boolean intakerConnected = true;
        public Measure<VoltageUnit> voltage = Volts.zero();
        public Measure<AngularVelocityUnit> intakerSpeed = RotationsPerSecond.zero();
        public Measure<CurrentUnit> intakerSupplyCurrent = Amps.zero();
        public boolean beamBreakState = false;
    }
}
