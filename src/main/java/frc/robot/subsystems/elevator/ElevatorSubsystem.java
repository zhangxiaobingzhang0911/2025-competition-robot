package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public ElevatorSubsystem(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    public Command setExtension(DoubleSupplier heightMeters) {
        return this.run(() -> {
            io.setTarget(heightMeters.getAsDouble());
            Logger.recordOutput("Elevator/Setpoint", heightMeters.getAsDouble());
        });
    }

    public Command setExtension(double meters) {
        return this.setExtension(() -> meters);
    }

    // TODO need test
    /*
     * ublic Command runCurrentZeroing() { return this.run( () -> {
     * io.setVoltage(-0.5); Logger.recordOutput("Elevator/Setpoint", Double.NaN); })
     * .until(() -> inputs.statorCurrentAmps[0] > 20.0) .finallyDo( (interrupted) ->
     * { if (!interrupted) { io.resetEncoder(0.0); hasZeroed = true; } }); }
     */

    public Command setVoltage(double voltage) {
        return this.run(() -> {
            io.setVoltage(voltage);
        });
    }

    public Command setVoltage(DoubleSupplier voltage) {
        return this.setVoltage(voltage.getAsDouble());
    }

    public double getExtensionMeters() {
        return inputs.positionMeters;
    }

    public boolean isNearExtension(double expected) {
        return MathUtil.isNear(expected, inputs.positionMeters, 0.02);
    }
}
