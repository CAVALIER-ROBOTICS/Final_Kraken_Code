// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystems.ArmAngleSubsystem;

public class ArmAngleCommand extends CommandBase {
  /** Creates a new ArmAngleCommand. */
  ArmAngleSubsystem armAngleSub;
  DoubleSupplier dubsup;

  public ArmAngleCommand(ArmAngleSubsystem a, DoubleSupplier d) {
    // Use addRequirements() here to declare subsystem dependencies.
    armAngleSub = a;
    dubsup = d;
    addRequirements(a);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(dubsup.getAsDouble()) >= .1) {
      double setpoint = (dubsup.getAsDouble() * -.4);
      armAngleSub.setPercentage(setpoint);
      SmartDashboard.putNumber("SetpointMotor", setpoint);
    } else {
      SmartDashboard.putBoolean("Setting", false);
      // armAngleSub.setPercentage(0.0);
      armAngleSub.stopArm();
    }

    // armAngleSub.setAnglePosition(-110);
    // SmartDashboard.putNumber("ArmAngle", armAngleSub.getAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armAngleSub.setPercentage(0);
    armAngleSub.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
