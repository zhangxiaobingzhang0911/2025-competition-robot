package frc.robot.auto.basics;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoAimShootCommand;
import frc.robot.commands.GroundIntakeCommand;
import frc.robot.commands.PreShootCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.drivers.DestinationSupplier;

import java.util.function.BooleanSupplier;

import static frc.robot.RobotConstants.ElevatorConstants.HOME_EXTENSION_METERS;
import static frc.robot.RobotConstants.ElevatorConstants.L4_EXTENSION_METERS;

public class AutoActions {
    private final EndEffectorSubsystem endEffectorSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final IndicatorSubsystem indicatorSubsystem;
    private final Swerve swerve;

    public AutoActions(IndicatorSubsystem indicatorSubsystem, ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem, IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.indicatorSubsystem = indicatorSubsystem;
        this.swerve = Swerve.getInstance();
    }

    public void invokeCommand(String name, BooleanSupplier stopSupplier) {
        switch (name) {
            case "DEPLOY-INTAKE":
                deployIntake().until(stopSupplier).schedule();
                break;
            case "PRESHOOT":
                preShoot().until(stopSupplier).schedule();
            case "AIMSHOOT":
                autoAimShoot().until(stopSupplier).schedule();
        }
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
        return new GroundIntakeCommand(indicatorSubsystem, intakeSubsystem, endEffectorSubsystem, elevatorSubsystem);
    }

    public Command preShoot() {
        return new PreShootCommand(indicatorSubsystem, endEffectorSubsystem, intakeSubsystem, elevatorSubsystem);
    }

    public Command autoAimShoot() {
        return new AutoAimShootCommand(indicatorSubsystem, endEffectorSubsystem, elevatorSubsystem, intakeSubsystem, () -> false);
    }

    public Command shootCoral() {
        return new ShootCommand(indicatorSubsystem, endEffectorSubsystem);
    }

    public Command shootCoralAtSetpoint() {
        return shootCoral().onlyIf(() -> elevatorSubsystem.getIo().isNearExtension(elevatorSubsystem.getWantedPosition()))
                .andThen(
                        new InstantCommand(() -> preShoot().cancel())
                );
    }
}