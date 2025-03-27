package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.RobotConstants.ClimberConstants;

import static frc.robot.RobotConstants.LOOPER_DT;

public class ClimberIOSim implements ClimberIO {
    private final SingleJointedArmSim climberSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1),
            ClimberConstants.CLIMBER_RATIO,
            0.07,
            0.4,
            Math.toRadians(0),
            Math.toRadians(1),
            true,
            0.0);
    private final ProfiledPIDController climberPid = new ProfiledPIDController(5.0, 0.0, 2.6,
            new TrapezoidProfile.Constraints(10.0, 10.0));
    private final ArmFeedforward climberFf = new ArmFeedforward(0.0, 0.3856, 0.543);

    private double targetPositionDeg = 0.0;

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        climberSim.update(LOOPER_DT);
        inputs.appliedVolts = 0.0;
        inputs.currentPositionDeg = Math.toDegrees(climberSim.getAngleRads());
        inputs.statorCurrentAmps = climberSim.getCurrentDrawAmps();
        inputs.supplyCurrentAmps = 0.0;
        inputs.targetPositionDeg = targetPositionDeg;
        inputs.tempCelsius = 0.0;
    }

    @Override
    public void setTargetPosition(double targetPositionDeg) {
        climberSim.setInputVoltage(MathUtil.clamp(
                climberPid.calculate(climberSim.getAngleRads(), targetPositionDeg)
                        + climberFf.calculate(climberPid.getSetpoint().position, climberPid.getSetpoint().velocity), -12, 12));
        this.targetPositionDeg = targetPositionDeg;
    }

    @Override
    public void resetPosition() {

    }

    @Override
    public void setCoast() {

    }

    @Override
    public void setBrake() {

    }
}