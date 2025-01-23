package frc.robot.subsystems.elevator;

import com.sun.source.tree.CaseTree;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Volt;

@Getter
public class ElevatorSubsystem extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final double startingPosition;

    public enum WantedState {
        IDLE,
        ANCHORED,
        MOVE_TO_TARGET
    }

    public enum SystemState {
        IDLING,
        ANCHORED,
        MOVING_TO_TARGET
    }

    private double[] targetPositions = new double[] {
            0,
            RobotConstants.ElevatorConstants.l1Position,
            RobotConstants.ElevatorConstants.l2Position,
            RobotConstants.ElevatorConstants.l3Position,
            RobotConstants.ElevatorConstants.l4Position
    };

    @Getter
    private SystemState systemState = SystemState.IDLING;

    @Getter @Setter
    private WantedState wantedState = WantedState.IDLE;

    @Setter
    private int targetLevel;

    public ElevatorSubsystem(ElevatorIO io) {
        this.io = io;
        this.startingPosition = this.getIo().getElevatorPosition();
        this.targetLevel = 0;
        for (int i = 0; i <= 5; i++) {
            this.targetPositions[i] += this.startingPosition;
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        SystemState newState = handleStateTransition();
        if (newState != this.systemState) {
            Logger.recordOutput("Elevator/SystemState", newState.toString());
            this.systemState = newState;
        }

        switch (this.systemState) {
            case IDLING, ANCHORED -> this.io.setElevatorDirectVoltage(Volt.of(0));
            case MOVING_TO_TARGET -> this.io.setElevatorPosition(this.targetPositions[this.targetLevel]);
        }
    }

    private SystemState handleStateTransition() {
        return switch (this.wantedState) {
            case MOVE_TO_TARGET -> {
                if (this.systemState == SystemState.ANCHORED) {
                    yield SystemState.ANCHORED;
                } else if (this.getIo().getElevatorPosition() == this.targetPositions[this.targetLevel]) {
                    yield SystemState.IDLING;
                }
                yield SystemState.MOVING_TO_TARGET;
            }
            case IDLE -> SystemState.IDLING;
            case ANCHORED -> SystemState.ANCHORED;
        };
    }
}
