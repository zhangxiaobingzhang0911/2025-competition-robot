package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;

public class Superstructure extends SubsystemBase {
    private EndEffectorSubsystem endEffector;

    public enum WantedSuperState {
        STOPPED
    }

    public enum CurrentSuperState {
        STOPPED
    }

    WantedSuperState wantedSuperState = WantedSuperState.STOPPED;
    CurrentSuperState currentSuperState = CurrentSuperState.STOPPED;
    CurrentSuperState previousSuperState;

    public Superstructure(
            EndEffectorSubsystem endEffector
    ) {
        this.endEffector = endEffector;
    }

    @Override
    public void periodic() {
        currentSuperState = handleStateTransition();
        applyStates();
    }

    private CurrentSuperState handleStateTransition() {
        previousSuperState = currentSuperState;
    }

}