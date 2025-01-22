package frc.robot.subsystems.intaker;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;

public class IntakerSubsystem extends SubsystemBase {
    private static double COLLECTING_RPM = RobotConstants.IntakerConstants.COLLECTING_RPM.get();
    private static double OUTTAKING_RPM = RobotConstants.IntakerConstants.OUTTAKING_RPM.get();
    private static double REJECTING_RPM = RobotConstants.IntakerConstants.REJECTING_RPM.get();
    private static double IDLING_RPM = RobotConstants.IntakerConstants.IDLING_RPM.get();
    private final IntakerIO io;
    private final IntakerIOInputsAutoLogged inputs = new IntakerIOInputsAutoLogged();
    private final BooleanSupplier beamBreakTripped;
    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLING;
    public IntakerSubsystem(IntakerIO io, BooleanSupplier isBeamBreakTripped) {
        this.io = io;
        this.beamBreakTripped = isBeamBreakTripped;
    }

    @Override
    public void periodic() {
        // read inputs
        io.updateInputs(inputs);
        // log inputs
        Logger.processInputs("Intaker", inputs);

        // process inputs
        SystemState newState = handleStateTransition();
        if (newState != systemState) {
            Logger.recordOutput("Intaker/SystemState", newState.toString());
            systemState = newState;
        }

        // holds the values to apply
        double intakerMotorRPM;

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            systemState = SystemState.IDLING;
        }

        // refresh RPM tunable numbers
        if (RobotConstants.TUNING) {
            COLLECTING_RPM = RobotConstants.IntakerConstants.COLLECTING_RPM.get();
            OUTTAKING_RPM = RobotConstants.IntakerConstants.OUTTAKING_RPM.get();
            REJECTING_RPM = RobotConstants.IntakerConstants.REJECTING_RPM.get();
            IDLING_RPM = RobotConstants.IntakerConstants.IDLING_RPM.get();
        }

        // set speeds based on state
        switch (systemState) {
            case EJECTING:
                intakerMotorRPM = OUTTAKING_RPM;
                break;
            case REJECTING:
                intakerMotorRPM = REJECTING_RPM;
                break;
            case COLLECTING:
                intakerMotorRPM = COLLECTING_RPM;
                break;
            case IDLING:
                intakerMotorRPM = IDLING_RPM;
                break;
            case OFF:
            default:
                intakerMotorRPM = 0.0;
                break;
        }

        // write outputs
        io.setVelocity(intakerMotorRPM / 60);


    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case OFF -> SystemState.OFF;
            case EJECT -> SystemState.EJECTING;
            case REJECT -> SystemState.REJECTING;
            case COLLECT -> {
                if (beamBreakTripped.getAsBoolean()) {
                    //TODO:whether rejecting state needed
                    yield SystemState.REJECTING;
                }
                yield SystemState.COLLECTING;
            }
            default -> SystemState.IDLING;
        };
    }

    public boolean isBeamBreakTripped() {
        return inputs.beamBreakState;
    }

    /**
     * Sets the target state for the intake subsystem.
     *
     * @param wantedState the target state for the intake subsystem.
     */
    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public enum WantedState {
        IDLE,
        COLLECT,
        REJECT,
        EJECT,
        OFF
    }

    public enum SystemState {
        IDLING,
        COLLECTING,
        REJECTING,
        EJECTING,
        OFF
    }
}
