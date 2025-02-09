package frc.robot.subsystems.elevator;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

@Getter
public class ElevatorSubsystem extends SubsystemBase {

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

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLING;
    private WantedState previousWantedState = WantedState.IDLE;

    private String shootPositionName = "Null";
    private double shootPosition = 0.0;

     private final LinearFilter currentFilter = LinearFilter.movingAverage(5);
    public double currentFilterValue = 0.0;

    private String wantedPositionType = "Null";
    private double wantedPosition = 0.0;

    public ElevatorSubsystem(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        Logger.recordOutput("Elevator/WantedState", wantedState.toString());
        Logger.recordOutput("Elevator/preiousWantedState", previousWantedState.toString());

        currentFilterValue = currentFilter.calculate(inputs.statorCurrentAmps[0]);
        Logger.recordOutput("Elevator/CurrentFilter", currentFilterValue);
        // process inputs
        SystemState newState = handleStateTransition();
        if (newState != systemState) {
            Logger.recordOutput("Elevator/SystemState", newState.toString());
            systemState = newState;
        }

        if (DriverStation.isDisabled()) {
            wantedState = wantedState.ZERO;
        }

        // set movements based on state
        switch (systemState) {
            case POSITION_GOING:
                io.setElevatorTarget(wantedPosition);
                break;
            case ZEROING:
                zeroElevator();
                break;
            case IDLING:
                io.setElevatorDirectVoltage(0);
                break;
            default:
                io.setElevatorDirectVoltage(0);
                break;
        }
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case ZERO -> {

                yield SystemState.ZEROING;
            }
            case POSITION -> {
                if(systemState == SystemState.ZEROING) {
                    wantedState = previousWantedState;
                    yield SystemState.ZEROING;
                }
                yield SystemState.POSITION_GOING;
            }
            case IDLE -> {
                if(systemState == SystemState.POSITION_GOING) {
                    wantedState = previousWantedState;
                    yield SystemState.POSITION_GOING;
                }
                yield SystemState.IDLING;
            }
            default -> {
                yield SystemState.IDLING;
            }
        };
    }

    public void setElevatorState(WantedState wantedState) {
        previousWantedState = this.wantedState;
        this.wantedState = wantedState;
    }
    public Command setElevatorStateCommand(WantedState wantedState) {
        return new InstantCommand(() -> setElevatorState(wantedState));
    }

    public void setElevatorShootPosition(String shootPositionName,double shootPosition) {
        this.shootPositionName = shootPositionName;
        this.shootPosition = shootPosition;
    }
    public Command setElevatorShootPositionCommand(String shootPositionName,double shootPosition){
        return new InstantCommand(() -> setElevatorShootPosition(shootPositionName,shootPosition));
    }

    public void setElevatorWantedPosition(String wantedPositionType) {
        this.wantedPositionType = wantedPositionType;
        switch (wantedPositionType) {
            case "Shoot" -> this.wantedPosition = this.shootPosition;
            case "Intaker Intake" -> this.wantedPosition = RobotConstants.ElevatorConstants.INTAKER_INTAKE_METERS.get();
            case "Funnel Intake" -> this.wantedPosition = RobotConstants.ElevatorConstants.FUNNEL_INTAKE_METERS.get();
            case "Intake Avoid" -> this.wantedPosition = RobotConstants.ElevatorConstants.INTAKER_AVOID_METERS.get();
        }
    }
    public Command setElevatorWantedPositionCommand(String wantedPositionType){
        return new InstantCommand(() -> setElevatorWantedPosition(wantedPositionType));
    }

    public double getLeaderCurrent(){
        return currentFilterValue;
    }
    public boolean isCurrentMax(){
        return Math.abs(this.getLeaderCurrent()) > RobotConstants.ElevatorConstants.ELEVATOR_ZEROING_CURRENT.get();
    }

    public void zeroElevator() {
        if(!isCurrentMax()) {
            io.setElevatorDirectVoltage(-3);
            wantedState = WantedState.ZERO;
        }
        if(isCurrentMax()){
            io.setElevatorDirectVoltage(0);
            io.resetElevatorPosition();
            wantedState = WantedState.IDLE;
        }
    }
}