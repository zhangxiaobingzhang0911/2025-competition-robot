package frc.robot.subsystems.intake;

import frc.robot.subsystems.roller.RollerIOReal;

import static frc.robot.RobotConstants.CANIVORE_CAN_BUS_NAME;
import static frc.robot.RobotConstants.intakeConstants.*;

public class IntakeRollerIOReal extends RollerIOReal implements IntakeRollerIO {
    private static final int id = INTAKE_MOTOR_ID;
    private static final String canbus = CANIVORE_CAN_BUS_NAME;
    private static final int statorCurrentLimitAmps = STATOR_CURRENT_LIMIT_AMPS;
    private static final int supplyCurrentLimitAmps = SUPPLY_CURRENT_LIMIT_AMPS;
    private static final boolean invert = IS_INVERT;
    private static final boolean brake = IS_BRAKE;

    public IntakeRollerIOReal() {
        super(id, canbus, statorCurrentLimitAmps, supplyCurrentLimitAmps, invert, brake);
    }
}
