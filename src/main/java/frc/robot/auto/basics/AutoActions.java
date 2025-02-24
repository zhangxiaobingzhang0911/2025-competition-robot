package frc.robot.auto.basics;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swerve.Swerve;

import java.util.HashMap;
import java.util.Map;

import static frc.robot.RobotConstants.ElevatorConstants.*;

public class AutoActions {
    private final EndEffectorSubsystem endEffectorSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final Swerve swerve;
    public boolean initialized = false;
    private Map<String, Command> autoCommands = new HashMap<>();

    public AutoActions(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem, IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.swerve = Swerve.getInstance();
        initializeAutoCommands();
    }

    public void initializeAutoCommands() {
        if (initialized) {
            throw new IllegalStateException("AutoActions already initialized");
        }
        autoCommands = Map.ofEntries(
                Map.entry(
                        "EE-GROUND-INTAKE", Commands.runOnce(() -> endEffectorSubsystem.setWantedState(EndEffectorSubsystem.WantedState.GROUND_INTAKE))
                ),
                Map.entry(
                        "EE-PRE-SHOOT", Commands.runOnce(() -> endEffectorSubsystem.setWantedState(EndEffectorSubsystem.WantedState.PRE_SHOOT))
                ),
                Map.entry(
                        "EE-IDLE", Commands.runOnce(() -> endEffectorSubsystem.setWantedState(EndEffectorSubsystem.WantedState.IDLE))
                ),
                Map.entry(
                        "EE-SHOOT", Commands.runOnce(() -> endEffectorSubsystem.setWantedState(EndEffectorSubsystem.WantedState.SHOOT))
                ),
                Map.entry(
                        "ELE-ZERO", Commands.runOnce(() -> elevatorSubsystem.setElevatorState(ElevatorSubsystem.WantedState.ZERO))
                ),
                Map.entry(
                        "ELE-L1", Commands.runOnce(() -> elevatorSubsystem.setElevatorPosition(L1_EXTENSION_METERS.get()))
                ),
                Map.entry(
                        "ELE-L2", Commands.runOnce(() -> elevatorSubsystem.setElevatorPosition(L2_EXTENSION_METERS.get()))
                ),
                Map.entry(
                        "ELE-L3", Commands.runOnce(() -> elevatorSubsystem.setElevatorPosition(L3_EXTENSION_METERS.get()))
                ),
                Map.entry(
                        "ELE-L4", Commands.runOnce(() -> elevatorSubsystem.setElevatorPosition(L4_EXTENSION_METERS.get()))
                ),
                Map.entry(
                        "ELE-HOME", Commands.runOnce(() -> elevatorSubsystem.setElevatorPosition(HOME_EXTENSION_METERS.get()))
                ),
                Map.entry(
                        "INTAKE-DEPLOY", Commands.runOnce(() -> intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.DEPLOY_INTAKE))
                ),
                Map.entry(
                        "INTAKE-HOME", Commands.runOnce(() -> intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.HOME))
                ),
                Map.entry(
                        "INTAKE-ZERO", Commands.runOnce(() -> intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.GROUNDZERO))
                )
        );
    }

    public void invokeCommand(String name) {
        assert autoCommands.containsKey(name);
        autoCommands.get(name).schedule();
    }

    public Command followPath(PathPlannerPath path, boolean angleLock, boolean requiredOnTarget, boolean resetOdometry) {
        return new FollowPath(this, swerve, path, angleLock, requiredOnTarget, resetOdometry);
    }

    public Command waitFor(double seconds) {
        return new WaitCommand(seconds);
    }

    public Command raiseElevator() {
        return Commands.runOnce(() -> elevatorSubsystem.setElevatorPosition(L4_EXTENSION_METERS.get()));
    }

    public Command deployIntake() {
        return Commands.runOnce(() -> intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.DEPLOY_INTAKE));
    }

    public Command preShoot() {
        return Commands.runOnce(() -> endEffectorSubsystem.setWantedState(EndEffectorSubsystem.WantedState.PRE_SHOOT));
    }

    public Command shootCoral() {
        return Commands.runOnce(() -> endEffectorSubsystem.setWantedState(EndEffectorSubsystem.WantedState.SHOOT));
    }

    public Command shootCoralAtSetpoint() {
        return preShoot().andThen(
                shootCoral().onlyIf(() -> elevatorSubsystem.getIo().isNearExtension(elevatorSubsystem.getWantedPosition())));
    }
}