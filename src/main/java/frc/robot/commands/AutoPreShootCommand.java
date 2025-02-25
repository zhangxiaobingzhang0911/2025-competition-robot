package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class AutoPreShootCommand extends Command {
    private final EndEffectorSubsystem endEffectorSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    Timer timer = new Timer();

    public AutoPreShootCommand(EndEffectorSubsystem endEffectorSubsystem, IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.HOME);
        elevatorSubsystem.setElevatorPosition(DestinationSupplier.getInstance().getElevatorSetpoint(true));
        endEffectorSubsystem.setWantedState(EndEffectorSubsystem.WantedState.PRE_SHOOT);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        System.out.print(timer.hasElapsed(0.1) + " ");
        System.out.print(elevatorSubsystem.elevatorReady(0.005) + " ");
        System.out.println(endEffectorSubsystem.isShootReady());

        return timer.hasElapsed(0.1) &&
                elevatorSubsystem.elevatorReady(0.005) &&
                endEffectorSubsystem.isShootReady();
    }
}
