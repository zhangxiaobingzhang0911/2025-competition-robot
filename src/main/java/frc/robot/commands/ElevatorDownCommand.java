package frc.robot.commands;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.*;

public class ElevatorDownCommand extends Command{
    private ElevatorSubsystem ElevatorSubsystem;
    public ElevatorDownCommand(ElevatorSubsystem ElevatorSubsystem){
        this.ElevatorSubsystem = ElevatorSubsystem;
        addRequirements(ElevatorSubsystem);
    }

    @Override
    public void execute(){
        ElevatorSubsystem.getIo().setElevatorDirectVoltage(RobotConstants.ElevatorConstants.elevatorDownVoltage);
    }
}