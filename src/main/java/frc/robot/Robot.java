package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swerve.Swerve;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

public class Robot extends LoggedRobot {
    private final Swerve swerve = Swerve.getInstance();
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        // logger initialization
        Logger.addDataReceiver(new NT4Publisher());
        // Logger.addDataReceiver(new WPILOGWriter());
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.start();

        // early-stage initialization
        DriverStation.silenceJoystickConnectionWarning(true);
        PowerDistribution PDP = new PowerDistribution();
        PDP.clearStickyFaults();
        PDP.close();

        robotContainer = new RobotContainer();
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
            autonomousCommand = robotContainer.getAutonomousCommand();
        } catch (Exception e) {
            System.out.println("Autonomous command failed: " + e);
            e.printStackTrace();
            autonomousCommand = null;
        }
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
        swerve.auto();
        robotContainer.getUpdateManager().invokeStart();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
        System.out.println("E1");
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
        System.out.println("E2");
        robotContainer.getUpdateManager().invokeStop();
        System.out.println("E3");
        swerve.normal();
        System.out.println("E4");
        swerve.cancelFollow();
        System.out.println("EF");
    }

    @Override
    public void teleopInit() {
        System.out.println("A1");
        swerve.normal();
        System.out.println("A2");
        robotContainer.getUpdateManager().invokeStart();
        System.out.println("AF");
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