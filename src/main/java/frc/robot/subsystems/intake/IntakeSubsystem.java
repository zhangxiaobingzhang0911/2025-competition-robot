package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.intakeConstants.intakeGainsClass;
import frc.robot.subsystems.beambreak.BeambreakIO;
import frc.robot.subsystems.beambreak.BeambreakIOInputsAutoLogged;
import frc.robot.subsystems.roller.RollerSubsystem;

import static frc.robot.RobotConstants.intakeConstants.*;

public class IntakeSubsystem extends RollerSubsystem{
    public static final String NAME = "Intake";
    private final IntakePivotIO intakePivotIO;
    private final IntakeRollerIO intakeRollerIO;
    private final BeambreakIO intakeBBIO;
    private BeambreakIOInputsAutoLogged intakeBBIOInputs = new BeambreakIOInputsAutoLogged();
    private IntakePivotIOInputsAutoLogged intakePivotIOInputs = new IntakePivotIOInputsAutoLogged();

    private WantedState wantedState = WantedState.RETRACT;
    private SystemState systemState = SystemState.RETRACTED;

    public double roller_kp = intakeGainsClass.INTAKE_KP.get();
    public double roller_ki = intakeGainsClass.INTAKE_KI.get();
    public double roller_kd = intakeGainsClass.INTAKE_KD.get();
    public double roller_ka = intakeGainsClass.INTAKE_KA.get();
    public double roller_kv = intakeGainsClass.INTAKE_KV.get();
    public double roller_ks = intakeGainsClass.INTAKE_KS.get();

    public static double PIVOT_RATIO = RobotConstants.intakeConstants.PIVOT_RATIO;

    private static double idleRPS = IDLE_RPS.get();
    private static double intakeRPS = INTAKE_RPS.get();
    private static double groundAngle = GROUND_ANGLE.get();
    private static double retractedAngle = RETRACTED_ANGLE.get();

    public IntakeSubsystem(
        IntakePivotIO intakePivotIO,
        IntakeRollerIO intakeRollerIO,
        BeambreakIO intakeBBIO
    ){
        super(intakeRollerIO, NAME);
        this.intakeBBIO = intakeBBIO;
        this.intakePivotIO = intakePivotIO;
        this.intakeRollerIO = intakeRollerIO;
    }

    @Override
    public void periodic() {
        super.periodic();

        intakeRollerIO.updateConfigs(roller_kp, roller_ki, roller_kd, roller_ka, roller_kv, roller_ks);
        intakeBBIO.updateInputs(intakeBBIOInputs);
        intakePivotIO.updateInputs(intakePivotIOInputs);

        SystemState newState = handleStateTransition();

        Logger.processInputs(NAME + "/Intake Beambreak", intakeBBIOInputs);
        Logger.processInputs(NAME + "/Pivot", intakePivotIOInputs);
        Logger.recordOutput("Intake/SystemState",newState.toString());

        if(newState!= systemState) {
            systemState = newState;
        }

        if(DriverStation.isDisabled()){
            systemState = SystemState.IDLING;
        }

        switch (systemState) {
            case INTAKING:
                io.setVelocity(intakeRPS);
                intakePivotIO.setMotorPosition(groundAngle);
                break;
            case OUTTAKING:
                io.setVelocity(-intakeRPS);
                intakePivotIO.setMotorPosition(groundAngle);
                break;
            case RETRACTED:
                io.setVelocity(idleRPS);
                intakePivotIO.setMotorPosition(retractedAngle);
                break;
            case IDLING:
                io.setVelocity(idleRPS);
                intakePivotIO.setMotorVoltage(0.0);
                break;
            case ZEROING:

            case OFF:
            default:
                io.setVelocity(idleRPS);
                break;
        }

        if (RobotConstants.TUNING) {
            intakeRPS = INTAKE_RPS.get();
            idleRPS = IDLE_RPS.get();
            groundAngle = GROUND_ANGLE.get();
            retractedAngle =RETRACTED_ANGLE.get();

            roller_ka = intakeGainsClass.INTAKE_KA.get();
            roller_kp = intakeGainsClass.INTAKE_KP.get();
            roller_ki = intakeGainsClass.INTAKE_KI.get();
            roller_kd = intakeGainsClass.INTAKE_KD.get();
            roller_ks = intakeGainsClass.INTAKE_KS.get();
            roller_kv = intakeGainsClass.INTAKE_KV.get();
        }
    };

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case INTAKE -> SystemState.INTAKING;
            case OUTTAKE -> SystemState.OUTTAKING;
            case RETRACT -> SystemState.RETRACTED;
            case IDLE -> SystemState.IDLING;
            case ZERO -> SystemState.ZEROING;
            case OFF -> SystemState.OFF;
            default -> SystemState.IDLING;
        };
    }

    public void setWantedState(WantedState wantedState) {this.wantedState = wantedState;}

    public enum WantedState{
        INTAKE,
        OUTTAKE,
        RETRACT,
        IDLE,
        ZERO,
        OFF
    }
    public enum SystemState{
        INTAKING,
        OUTTAKING,
        RETRACTED,
        IDLING,
        ZEROING,
        OFF
    }
}