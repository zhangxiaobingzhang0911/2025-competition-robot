package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.roller.RollerIO;
import frc.robot.subsystems.roller.RollerSubsystem;

public class IntakeRollerSubsystem extends RollerSubsystem {
    private static double INTAKE_VELOCITY = RobotConstants.intakeConstants.INTAKE_VELOCITY.get();
    private static double INTAKE_VOLTAGE = RobotConstants.intakeConstants.INTAKE_VOLTAGE.get();
    // TODO test whether by velocity or voltage

    @Override
    public void periodic() {
        if (RobotConstants.TUNING) {
            INTAKE_VELOCITY = RobotConstants.intakeConstants.INTAKE_VELOCITY.get();
            INTAKE_VOLTAGE = RobotConstants.intakeConstants.INTAKE_VOLTAGE.get();
        }
    }

    public IntakeRollerSubsystem(RollerIO io) {
        super(io, "Intake/Roller");
    }

    public Command velocityIntake() {
        return velocityIntake(INTAKE_VELOCITY);
    }

    public Command velocityIntake(double velocity) {
        return setVelocity(velocity);
    }

    public Command velocityOuttake() {
        return velocityIntake(-INTAKE_VELOCITY);
    }

    public Command voltageIntake() {
        return voltageIntake(INTAKE_VOLTAGE);
    }

    public Command voltageIntake(double voltage) {
        return setVoltage(voltage);
    }

    public Command voltageOuttake() {
        return velocityIntake(-INTAKE_VOLTAGE);
    }
}