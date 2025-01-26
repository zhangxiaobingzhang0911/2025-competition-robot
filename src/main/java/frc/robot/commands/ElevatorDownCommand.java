package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ElevatorDownCommand extends Command{
    private ElevatorSubsystem elevatorSubsystem;
    public ElevatorDownCommand(ElevatorSubsystem elevatorSubsystem){
        this.elevatorSubsystem = elevatorSubsystem;

    }

    @Override
    public void execute(){
        elevatorSubsystem.getIo().setElevatorDirectVoltage(1);
    }
}