package frc.robot.commands;

import java.util.concurrent.TimeUnit;

import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

// This class represents a command that makes specified HID devices rumble for a defined duration
public class RumbleCommand extends Command {
 private final GenericHID[] hid;
 private final Timer timer = new Timer();
 private final Measure<edu.wpi.first.units.TimeUnit> rumbleTime;

 // Constructor for RumbleCommand, initializes the HID devices and the duration of the rumble
 public RumbleCommand(Measure<edu.wpi.first.units.TimeUnit> seconds, GenericHID... hid) {
  this.hid = hid;
  this.rumbleTime = seconds;
 }

 // Called when the command is initially scheduled. Starts the timer and sets all HID devices to rumble
 @Override
 public void initialize() {
  timer.restart();
  for (var i : hid) {
   i.setRumble(GenericHID.RumbleType.kBothRumble, 1);
  }
 }

 // Called once the command ends or is interrupted. Stops the rumble on all HID devices
 @Override
 public void end(boolean interrupted) {
  for (var i : hid) {
   i.setRumble(GenericHID.RumbleType.kBothRumble, 0);
  }
 }

 // Returns true if the command has finished executing, based on the elapsed time
 @Override
 public boolean isFinished() {
  return timer.hasElapsed(rumbleTime.magnitude());
 }
}