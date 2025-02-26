package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.TunableNumber;
import org.littletonrobotics.junction.Logger;

//TODO: change motion logic and reset command afterwards

public class ClimberSubsystem extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    private final TunableNumber deployAngle = new TunableNumber("CLIMBER/deployAngle", 0);
    private final TunableNumber climbAngle = new TunableNumber("CLIMBER/climbAngle", -1350);
    private WantedState wantedState = WantedState.DEPLOY;
    private SystemState systemState = SystemState.DEPLOYING;

    public ClimberSubsystem(ClimberIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        SystemState newState = handleStateTransition();

        Logger.processInputs("Climber", inputs);
        Logger.recordOutput("Climber/SystemState", newState.toString());

        if (newState != systemState) {
            systemState = newState;
        }

        switch (systemState) {
            case DEPLOYING:
                io.setTargetPosition(deployAngle.get());
                break;
            case CLIMBING:
                io.setTargetPosition(climbAngle.get());
                break;
        }
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case DEPLOY -> SystemState.DEPLOYING;
            case CLIMB -> SystemState.CLIMBING;
        };
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public void resetPosition() {
        io.resetPosition();
    }

    public void setCoast() {
        io.setCoast();
    }

    public void setBrake() {
        io.setBrake();
    }

    public enum WantedState {
        DEPLOY,
        CLIMB
    }

    public enum SystemState {
        DEPLOYING,
        CLIMBING
    }


}
