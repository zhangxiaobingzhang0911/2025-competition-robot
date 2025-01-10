// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.RobotConstants.OperatorConstants;
import frc.robot.Utils.AllianceFlipUtil;
import frc.robot.auto.basics.AutoActions;
import frc.robot.display.Display;
import frc.robot.subsystems.SwerveSubsystem.SwerveSubsystem;
import lombok.Getter;

import java.io.IOException;

import org.frcteam6941.Looper.UpdateManager;
import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  CommandXboxController driverController = new CommandXboxController(0);
  CommandXboxController operatorController = new CommandXboxController(1);
  Display display = Display.getInstance();

  @Getter
    private UpdateManager updateManager;
  // The robot's subsystems and commands are defined here...

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
      updateManager = new UpdateManager(swerveSubsystem,
                display);
        updateManager.registerAll();

        configureBindings();
        System.out.println("Init Completed!");

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    swerveSubsystem.setDefaultCommand(Commands
                .runOnce(() -> swerveSubsystem.drive(
                                new Translation2d(
                                        -RobotConstants.driverController.getLeftY()
                                                * RobotConstants.SwerveConstants.maxSpeed.magnitude(),
                                        -RobotConstants.driverController.getLeftX()
                                                * RobotConstants.SwerveConstants.maxSpeed.magnitude()),
                                -RobotConstants.driverController.getRightX()
                                        * RobotConstants.SwerveConstants.maxAngularRate.magnitude(),
                                true,
                                false),
                        swerveSubsystem));

        // initial
        RobotConstants.driverController.start().onTrue(
                Commands.runOnce(() -> {
                    swerveSubsystem.resetHeadingController();
                    swerveSubsystem.resetPose(
                            new Pose2d(
                                    AllianceFlipUtil.apply(
                                            new Translation2d(0, 0)),
                                    Rotation2d.fromDegrees(
                                            swerveSubsystem.getLocalizer().getLatestPose().getRotation().getDegrees())));
                }).ignoringDisable(true));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   * @throws ParseException 
   * @throws IOException 
   * @throws FileVersionException 
   */
  public Command getAutonomousCommand() throws FileVersionException, IOException, ParseException {
    // An example command will be run in autonomous
    return new SequentialCommandGroup(
                AutoActions.waitFor(0.000001),
                AutoActions.followTrajectory(AutoActions.getTrajectory("T_4"), true, true)
        );
  }
}
