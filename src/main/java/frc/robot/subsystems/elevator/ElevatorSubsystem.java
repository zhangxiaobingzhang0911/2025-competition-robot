package frc.robot.subsystems.elevator;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import static frc.robot.RobotContainer.elevatorIsDanger;

public class ElevatorSubsystem extends SubsystemBase {
    @Getter
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final LinearFilter currentFilter = LinearFilter.movingAverage(5);
    public double currentFilterValue = 0.0;
    private WantedState wantedState = WantedState.POSITION;
    private SystemState systemState = SystemState.POSITION_GOING;
    private double targetPosition = 0.5;
    private boolean hasReachedNearZero = false;

    public ElevatorSubsystem(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        currentFilterValue = currentFilter.calculate(inputs.statorCurrentAmps);
        Logger.recordOutput("Elevator/CurrentFilter", currentFilterValue);

        Logger.recordOutput("Elevator/isNear", io.isNearExtension(targetPosition));
        Logger.recordOutput("Elevator/isNearZero", io.isNearZeroExtension());
        Logger.recordOutput("Elevator/setPoint", targetPosition);
        Logger.recordOutput("Elevator/wantedState", wantedState.toString());

        elevatorIsDanger = elevatorIsDanger();
        Logger.recordOutput("Flags/elevatorIsDanger", elevatorIsDanger());

        SystemState newState = handleStateTransition();
        if (newState != systemState) {
            systemState = newState;
        }

        Logger.recordOutput("Elevator/SystemState", systemState.toString());

        switch (systemState) {
            case POSITION_GOING:
                io.setElevatorTarget(targetPosition);
                break;
            case ZEROING:
                zeroElevator();
                break;
            //TODO verify utility, may delete
            case IDLING:
                io.setElevatorVoltage(0);
                break;
            default:
                throw new IllegalArgumentException("Unknown systemState: " + systemState);
        }
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case ZERO -> SystemState.ZEROING;
            case POSITION -> {
                if (systemState == SystemState.ZEROING) {
                    wantedState = WantedState.ZERO;
                    yield SystemState.ZEROING;
                }
                yield SystemState.POSITION_GOING;
            }
            case IDLE -> {
                if (systemState == SystemState.POSITION_GOING) {
                    wantedState = WantedState.ZERO;
                    yield SystemState.POSITION_GOING;
                }
                yield SystemState.IDLING;
            }
        };
    }

    public void setElevatorState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public void setElevatorPosition(double position) {
        targetPosition = position;
        setElevatorState(WantedState.POSITION);
    }

    public void zeroElevator() {
        if (!io.isNearZeroExtension() && !hasReachedNearZero) {
            io.setElevatorTarget(0.05);
            return;
        }
        hasReachedNearZero = true;
        if (currentFilterValue <= RobotConstants.ElevatorConstants.ELEVATOR_ZEROING_CURRENT.get()) {
            io.setElevatorVoltage(-1);
            wantedState = WantedState.ZERO;
        }
        if (currentFilterValue > RobotConstants.ElevatorConstants.ELEVATOR_ZEROING_CURRENT.get()) {
            io.setElevatorVoltage(0);
            io.resetElevatorPosition();
            wantedState = WantedState.IDLE;
            hasReachedNearZero = false;
        }
    }

    public boolean elevatorIsDanger() {
        return (inputs.positionMeters < 0.5);
    }

    public enum WantedState {
        POSITION,
        ZERO,
        IDLE
    }

    public enum SystemState {
        POSITION_GOING,
        ZEROING,
        IDLING
    }
}
