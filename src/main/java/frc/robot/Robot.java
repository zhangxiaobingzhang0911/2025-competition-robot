package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SuperstructureVisualizer;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

public class Robot extends LoggedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;
    private SuperstructureVisualizer currentStateVisualizer;

    @Override
    public void robotInit() {
        // logger initialization
        Logger.addDataReceiver(new NT4Publisher());
        // Logger.addDataReceiver(new WPILOGWriter());
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.start();
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

        // early-stage initialization
        DriverStation.silenceJoystickConnectionWarning(true);
        PowerDistribution PDP = new PowerDistribution();
        PDP.clearStickyFaults();
        PDP.close();
        // Camera used by driver to help aiming
        // Remember to adjust fps & resolution in elastic (5fps, 300*200)
        // If network is bad or rio is in high cpu usage, annotate the following line:
        CameraServer.startAutomaticCapture("Driver Camera", "/dev/video0");

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
            autonomousCommand = null;
        }

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
        robotContainer.getUpdateManager().invokeStart();
        // swerve.auto();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
        robotContainer.getUpdateManager().invokeStop();
        // swerve.normal();
        // swerve.cancelFollow();
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
        // swerve.normal();
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