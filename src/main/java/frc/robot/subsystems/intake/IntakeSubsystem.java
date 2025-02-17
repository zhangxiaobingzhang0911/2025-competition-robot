package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.roller.RollerSubsystem;
import org.littletonrobotics.junction.Logger;

import static frc.robot.RobotConstants.ElevatorConstants.ELEVATOR_MIN_SAFE_HEIGHT;
import static frc.robot.RobotConstants.IntakeConstants.*;

public class IntakeSubsystem extends RollerSubsystem {
    public static final String NAME = "Intake/Roller";
    private static double deployAngle = DEPLOY_ANGLE.get();
    private static double funnelAvoidAngle = FUNNEL_AVOID_ANGLE.get();
    private static double homeAngle = HOME_ANGLE.get();
    private static double intakeVoltage = INTAKE_VOLTAGE.get();
    private final IntakePivotIO intakePivotIO;
    private final IntakeRollerIO intakeRollerIO;
    private final IntakePivotIOInputsAutoLogged intakePivotIOInputs = new IntakePivotIOInputsAutoLogged();
    private WantedState wantedState = WantedState.HOME;
    private SystemState systemState = SystemState.HOMING;
    private boolean hasHomed = false;

    public IntakeSubsystem(
            IntakePivotIO intakePivotIO,
            IntakeRollerIO intakeRollerIO
    ) {
        super(intakeRollerIO, NAME);
        this.intakePivotIO = intakePivotIO;
        this.intakeRollerIO = intakeRollerIO;
    }

    @Override
    public void periodic() {
        super.periodic();

        intakePivotIO.updateInputs(intakePivotIOInputs);

        SystemState newState = handleStateTransition();

        Logger.processInputs("Intake/Pivot", intakePivotIOInputs);

        Logger.recordOutput("Intake/SystemState", systemState.toString());

        RobotContainer.intakeIsDanger = intakeIsDanger();
        RobotContainer.intakeIsAvoiding = intakeIsAvoiding();
        Logger.recordOutput(NAME + "/isnear", isNearAngle(FUNNEL_AVOID_ANGLE.get()));
        Logger.recordOutput("Flags/intakeIsDanger", intakeIsDanger());

        if (newState != systemState) {
            systemState = newState;
        }

        switch (systemState) {
            case DEPLOY_WITHOUT_ROLLING:
                intakeRollerIO.stop();
                intakePivotIO.setPivotAngle(deployAngle);
                break;
            case DEPLOY_INTAKING:
                intakeRollerIO.setVoltage(intakeVoltage);
                intakePivotIO.setPivotAngle(deployAngle);
                break;
            case TREMBLE_INTAKING:
                trembleIntake();
                break;
            case OUTTAKING:
                intakeRollerIO.setVoltage(-3);
                intakePivotIO.setPivotAngle(deployAngle);
                break;
            case FUNNEL_AVOIDING:
                intakeRollerIO.stop();
                intakePivotIO.setPivotAngle(funnelAvoidAngle);
                break;
            case HOMING:
                intakeRollerIO.stop();
                intakePivotIO.setPivotAngle(homeAngle);
                break;
            case GROUNDZEROING:
                zeroIntakeGround();
                break;
            case OFF:
        }

        if (RobotConstants.TUNING) {
            deployAngle = DEPLOY_ANGLE.get();
            funnelAvoidAngle = FUNNEL_AVOID_ANGLE.get();
            homeAngle = HOME_ANGLE.get();
            intakeVoltage = INTAKE_VOLTAGE.get();
        }
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case DEPLOY_WITHOUT_ROLL -> SystemState.DEPLOY_WITHOUT_ROLLING;
            case DEPLOY_INTAKE -> SystemState.DEPLOY_INTAKING;
            case TREMBLE_INTAKE -> SystemState.TREMBLE_INTAKING;
            case OUTTAKE -> SystemState.OUTTAKING;
            case FUNNEL_AVOID -> SystemState.FUNNEL_AVOIDING;
            case HOME -> {
                if (RobotContainer.elevatorIsDanger) {
                    yield SystemState.FUNNEL_AVOIDING;
                } else {
                    yield SystemState.HOMING;
                }
            }
            case GROUNDZERO -> SystemState.GROUNDZEROING;
            case OFF -> SystemState.OFF;
        };
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public void trembleIntake() {
        intakeRollerIO.setVoltage(intakeVoltage);
        intakePivotIO.setPivotAngle(deployAngle - 3);
        if (intakePivotIOInputs.currentAngleDeg > deployAngle + 2) {
            intakePivotIO.setPivotAngle(deployAngle - 3);
        } else if (intakePivotIOInputs.currentAngleDeg < deployAngle - 2) {
            intakePivotIO.setPivotAngle(deployAngle + 3);
        }

    }

    public void zeroIntakeGround() {
        intakeRollerIO.stop();
        if (!isNearAngle(90) && !hasHomed) {
                intakePivotIO.setPivotAngle(90);
                return;
        }
        hasHomed = true;
        if (intakePivotIOInputs.statorCurrentAmps <= 15) {
            intakePivotIO.setMotorVoltage(1);
            setWantedState(WantedState.GROUNDZERO);
        }
        if (intakePivotIOInputs.statorCurrentAmps > 15) {
            intakePivotIO.setMotorVoltage(0);
            intakePivotIO.resetAngle(120);
            setWantedState(WantedState.DEPLOY_WITHOUT_ROLL);
            hasHomed = false;
        }
    }




    public boolean isNearAngle(double targetAngleDeg) {
        return MathUtil.isNear(targetAngleDeg, intakePivotIOInputs.currentAngleDeg, 1);
    }

    public boolean intakeIsDanger() {
        return intakePivotIOInputs.currentAngleDeg < INTAKE_DANGER_ZONE;
    }

    private boolean intakeIsAvoiding() {
        return intakePivotIOInputs.currentAngleDeg > 50;
    }

    public enum WantedState {
        DEPLOY_WITHOUT_ROLL,
        DEPLOY_INTAKE,
        TREMBLE_INTAKE,
        OUTTAKE,
        FUNNEL_AVOID,
        HOME,
        GROUNDZERO,
        OFF
    }

    public enum SystemState {
        DEPLOY_WITHOUT_ROLLING,
        DEPLOY_INTAKING,
        TREMBLE_INTAKING,
        OUTTAKING,
        FUNNEL_AVOIDING,
        HOMING,
        GROUNDZEROING,
        OFF
    }
}