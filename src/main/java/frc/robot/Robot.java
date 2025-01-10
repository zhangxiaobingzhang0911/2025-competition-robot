package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SwerveSubsystem.SwerveSubsystem;

import java.io.IOException;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import com.pathplanner.lib.util.FileVersionException;

public class Robot extends LoggedRobot {
    SwerveSubsystem swerve = SwerveSubsystem.getInstance();
    private Command m_autonomousCommand;
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        Logger.addDataReceiver(new NT4Publisher());
        //Logger.addDataReceiver(new WPILOGWriter());
        Logger.start();
        robotContainer = new RobotContainer();
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        robotContainer.getUpdateManager().runEnableSingle();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        try {
          m_autonomousCommand = robotContainer.getAutonomousCommand();
        } catch (FileVersionException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        } catch (IOException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        } catch (ParseException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
        robotContainer.getUpdateManager().invokeStart();
        swerve.auto();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
        robotContainer.getUpdateManager().invokeStop();
        swerve.normal();
        swerve.cancelFollow();
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        swerve.normal();
        robotContainer.getUpdateManager().invokeStart();

    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void teleopExit() {
        robotContainer.getUpdateManager().invokeStop();

    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
        robotContainer.getUpdateManager().runSimulateSingle();
    }
}
