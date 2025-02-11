package frc.robot.subsystems.endeffector;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotConstants;
import frc.robot.subsystems.beambreak.BeambreakIO;
import frc.robot.subsystems.beambreak.BeambreakIOInputsAutoLogged;
import frc.robot.subsystems.roller.RollerSubsystem;

import static frc.robot.RobotConstants.EndEffectorConstants.*;
import static frc.robot.RobotConstants.EndEffectorConstants.EndEffectorGainsClass.*;

public class EndEffectorSubsystem extends RollerSubsystem {

    public static final String NAME = "EndEffector";

    private final EndEffectorIO endEffectorIO;
    private final BeambreakIO middleBBIO, edgeBBIO;
    private BeambreakIOInputsAutoLogged middleBBInputs = new BeambreakIOInputsAutoLogged();
    private BeambreakIOInputsAutoLogged edgeBBInputs = new BeambreakIOInputsAutoLogged();

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLING;

    private boolean hasTransitionedToTransfer = false;
    private boolean hasTransitionedToHold = false;

    public double kp = ENDEFFECTOR_KP.get();
    public double ki = ENDEFFECTOR_KI.get();
    public double kd = ENDEFFECTOR_KD.get();
    public double ka = ENDEFFECTOR_KA.get();
    public double kv = ENDEFFECTOR_KV.get();
    public double ks = ENDEFFECTOR_KS.get();

    private static double idleRPS = IDLE_RPS.get();
    private static double intakeRPS = INTAKE_RPS.get();
    private static double transferRPS = TRANSFER_RPS.get();
    private static double holdRPS = HOLD_RPS.get();
    private static double shootRPS = SHOOT_RPS.get();

    public enum WantedState {
        IDLE,
        FUNNEL_INTAKE,
        GROUND_INTAKE,
        TRANSFER,
        HOLD,
        SHOOT,
        OFF
    }

    public enum SystemState {
        IDLING,
        FUNNEL_INTAKING,
        GROUND_INTAKING,
        TRANSFERRING,
        HOLDING,
        SHOOTING,
        OFF
    }

    public EndEffectorSubsystem(EndEffectorIO endEffectorIO , BeambreakIO middleBBIO, BeambreakIO edgeBBIO) {
        super(endEffectorIO, NAME);
        this.endEffectorIO = endEffectorIO;
        this.middleBBIO = middleBBIO;
        this.edgeBBIO = edgeBBIO;
    }

    @Override
    public void periodic() {
        super.periodic();

        endEffectorIO.updateConfigs(kp, ki, kd, ka, kv, ks);
        middleBBIO.updateInputs(middleBBInputs);
        edgeBBIO.updateInputs(edgeBBInputs);

        SystemState newState = handleStateTransition();

        Logger.processInputs(NAME + "/Middle Beambreak", middleBBInputs);
        Logger.processInputs(NAME + "/Edge Beambreak", edgeBBInputs);
        Logger.recordOutput("EndEffector/SystemState", systemState.toString());
        Logger.recordOutput(NAME+"/wantedState",wantedState.toString());

        if (newState != systemState) {
            systemState = newState;
        }

        if (DriverStation.isDisabled()) {
            systemState = SystemState.IDLING;
        }

        switch(systemState) {
            case IDLING:
                io.setVelocity(idleRPS);
                break;
            case FUNNEL_INTAKING:
                io.setVelocity(intakeRPS);
                break;
            case GROUND_INTAKING:
                io.setVelocity(-intakeRPS);
                break;
            case TRANSFERRING:
                io.setVelocity(transferRPS);
                break;
            case HOLDING:
                io.setVelocity(holdRPS);
                break;
            case SHOOTING:
                io.setVelocity(shootRPS);
                break;
            case OFF:
            default:
                io.setVelocity(0.0);
                break;
        }

        if (RobotConstants.TUNING) {
            intakeRPS = INTAKE_RPS.get();
            holdRPS = HOLD_RPS.get();
            transferRPS = TRANSFER_RPS.get();
            shootRPS = SHOOT_RPS.get();

            kp = ENDEFFECTOR_KP.get();
            ki = ENDEFFECTOR_KI.get();
            kd = ENDEFFECTOR_KD.get();
            ka = ENDEFFECTOR_KA.get();
            kv = ENDEFFECTOR_KV.get();
            ks = ENDEFFECTOR_KS.get();
        }
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case IDLE -> SystemState.IDLING;
            case FUNNEL_INTAKE -> {
                if (middleBBInputs.isBeambreakOn) {
                    hasTransitionedToTransfer = true;
                    wantedState = WantedState.TRANSFER;
                    yield SystemState.TRANSFERRING;
                }
                yield SystemState.FUNNEL_INTAKING;
            }
            case GROUND_INTAKE -> {
                if (middleBBInputs.isBeambreakOn) {
                    wantedState = WantedState.TRANSFER;
                    hasTransitionedToTransfer = true;
                    yield SystemState.TRANSFERRING;
                }
                yield SystemState.GROUND_INTAKING;
            }
            case TRANSFER -> {
                if (edgeBBInputs.isBeambreakOn && !middleBBInputs.isBeambreakOn) {
                    wantedState = WantedState.HOLD;
                    hasTransitionedToHold = true;
                    yield SystemState.HOLDING;
                }
                yield SystemState.TRANSFERRING;
            }
            case HOLD -> {
                hasTransitionedToHold = true;
                yield SystemState.HOLDING;
            }
            case SHOOT -> {
                if (isShootFinished()) {
                    hasTransitionedToTransfer = false;
                    hasTransitionedToHold = false;
                    wantedState = WantedState.IDLE;
                    yield SystemState.IDLING;
                }
                yield SystemState.SHOOTING;
            }
            case OFF -> SystemState.OFF;
            default -> SystemState.IDLING;
        };
    }

    public boolean isCoralReady () {
        return hasTransitionedToHold;
    }

    public boolean isShootFinished () {
        return hasTransitionedToHold && !edgeBBInputs.isBeambreakOn;
    }

    public boolean isIntakeFinished () {
        return hasTransitionedToTransfer;
    }

    public boolean isEndEffectorIntaking () {
        return systemState == SystemState.FUNNEL_INTAKING || systemState==SystemState.GROUND_INTAKING || systemState==SystemState.TRANSFERRING;
    }

    public boolean isEndEffectorHolding () {
        return systemState == SystemState.HOLDING;
    }

    public void setWantedState(WantedState wantedState) {this.wantedState = wantedState;}

    public Command setWantedStateCommand(WantedState wantedState) {
        return new InstantCommand(() -> setWantedState(wantedState));
    }
}
