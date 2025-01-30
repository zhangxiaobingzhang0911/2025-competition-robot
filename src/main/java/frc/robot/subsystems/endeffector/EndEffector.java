package frc.robot.subsystems.endeffector;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;

import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {

    private final EndEffectorIO io;
    private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

    private SystemState systemState = SystemState.IDLING;
    private WantedState wantedState = WantedState.IDLE;

    private static double idlingRPS = RobotConstants.EndEffectorConstants.idlingRPS.get();
    private static double intakingVoltage = RobotConstants.EndEffectorConstants.intakingVoltage.get();
    private static double outakingVoltage = RobotConstants.EndEffectorConstants.outtakingVoltage.get();
    private static double indexingRPS = RobotConstants.EndEffectorConstants.indexingRPS.get();
    private static double preShootingRPS = RobotConstants.EndEffectorConstants.preShootingRPS.get();
    private static double shootingVoltage = RobotConstants.EndEffectorConstants.ShootingVoltage.get();

    public enum WantedState {
        IDLE,
        INTAKE,
        OUTTAKE,
        INDEX,
        PRE_SHOOT,
        SHOOT
    }

    public enum SystemState {
        IDLING,
        INTAKING,
        OUTTAKING,
        INDEXING,
        PRE_SHOOTING,
        SHOOTING
    }

    public EndEffector(EndEffectorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        // read inputs
        io.updateInputs(inputs);
        // log inputs
        Logger.processInputs("EndEffector", inputs);

        // process inputs
        SystemState newState = handleStateTransition();
        if (newState != systemState) {
            Logger.recordOutput("EndEffector/SystemState", newState.toString());
            systemState = newState;
        }

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            systemState = SystemState.IDLING;
        }

        // refresh RPM tunable numbers
        if (RobotConstants.TUNING) {
            idlingRPS = RobotConstants.EndEffectorConstants.idlingRPS.get();
            intakingVoltage = RobotConstants.EndEffectorConstants.intakingVoltage.get();
            outakingVoltage = RobotConstants.EndEffectorConstants.outtakingVoltage.get();
            indexingRPS = RobotConstants.EndEffectorConstants.indexingRPS.get();
            preShootingRPS = RobotConstants.EndEffectorConstants.preShootingRPS.get();
            shootingVoltage = RobotConstants.EndEffectorConstants.ShootingVoltage.get();
        }

        // set speeds based on state
        switch (systemState) {
            case IDLING:
                io.setVelocity(idlingRPS);
                break;
            case INTAKING:
                io.setVoltage(intakingVoltage);
                break;
            case OUTTAKING:
                io.setVoltage(outakingVoltage);
                break;
            case INDEXING:
                io.setVelocity(indexingRPS);
                break;
            case PRE_SHOOTING:
                io.setVelocity(preShootingRPS);
                break;
            case SHOOTING:
                io.setVoltage(shootingVoltage);
                break;
            default:
                io.setVelocity(0.0);
                break;
        }
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    // TODO simplify handleStateTransition logics after finishing superstructure
    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case INTAKE -> {
                if (inputs.islowerEndEffectorBeamBreakOn) {
                    yield SystemState.INDEXING;
                }
                yield SystemState.INTAKING;
            }
            case OUTTAKE -> SystemState.OUTTAKING;
            case INDEX -> {
                if (inputs.isHigherEndEffectorBeamBreakOn) {
                    yield SystemState.PRE_SHOOTING;
                }
                yield SystemState.INDEXING;
            }
            case PRE_SHOOT -> SystemState.PRE_SHOOTING;
            case SHOOT -> {
                if (!inputs.isHigherEndEffectorBeamBreakOn) {
                    yield SystemState.IDLING;
                }
                yield SystemState.SHOOTING;
            }
            default -> SystemState.IDLING;
        };
    }
}
