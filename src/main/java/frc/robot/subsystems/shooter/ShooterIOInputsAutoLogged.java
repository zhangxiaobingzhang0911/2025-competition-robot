package frc.robot.subsystems.shooter;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ShooterIOInputsAutoLogged extends ShooterIO.ShooterIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("ShooterVelocity", ShooterVelocity);
    table.put("ShooterPosition", ShooterPosition);
    table.put("ShooterAppliedVoltage", ShooterAppliedVoltage);
    table.put("ShooterShooterSupplyCurrent", ShooterSupplyCurrent);
    table.put("TargetShooterVelocity", targetShooterVelocity);
    table.put("ShooterKP", ShooterKP);
    table.put("ShooterKI", ShooterKI);
    table.put("ShooterKD", ShooterKD);
    table.put("ShooterKA", ShooterKA);
    table.put("ShooterKV", ShooterKV);
    table.put("ShooterKS", ShooterKS);
  }

  @Override
  public void fromLog(LogTable table) {
    ShooterAppliedVoltage = table.get("ShooterAppliedVoltage", ShooterAppliedVoltage);
    ShooterVelocity = table.get("ShooterVelocity", ShooterVelocity);
    ShooterSupplyCurrent = table.get("ShooterAppliedVoltage", ShooterSupplyCurrent);
    ShooterPosition = table.get("ShooterPosition", ShooterPosition);
    targetShooterVelocity = table.get("TargetShooterVelocity", targetShooterVelocity);
    ShooterKP = table.get("ShooterKP", ShooterKP);
    ShooterKI = table.get("ShooterKI", ShooterKI);
    ShooterKD = table.get("ShooterKD", ShooterKD);
    ShooterKA = table.get("ShooterKA", ShooterKA);
    ShooterKV = table.get("ShooterKV", ShooterKV);
    ShooterKS = table.get("ShooterKS", ShooterKS);
  }

  public ShooterIOInputsAutoLogged clone() {
    ShooterIOInputsAutoLogged copy = new ShooterIOInputsAutoLogged();
    copy.ShooterAppliedVoltage = this.ShooterAppliedVoltage;
    copy.ShooterPosition = this.ShooterPosition;
    copy.ShooterSupplyCurrent = this.ShooterSupplyCurrent;
    copy.ShooterVelocity = this.ShooterVelocity;
    copy.targetShooterVelocity = this.targetShooterVelocity;
    copy.ShooterKP = this.ShooterKP;
    copy.ShooterKI = this.ShooterKI;
    copy.ShooterKD = this.ShooterKD;
    copy.ShooterKA = this.ShooterKA;
    copy.ShooterKV = this.ShooterKV;
    copy.ShooterKS = this.ShooterKS;
    return copy;
  }
}
