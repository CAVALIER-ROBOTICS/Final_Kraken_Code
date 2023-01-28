// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystems.ArmExtendSubsystem;

public class ArmExtendCommand extends CommandBase {
  /** Creates a new ArmAngleCommand. */
  ArmExtendSubsystem armExtendSub;
  DoubleSupplier dubsup;

  public ArmExtendCommand(ArmExtendSubsystem a, DoubleSupplier d) {
    // Use addRequirements() here to declare subsystem dependencies.
    armExtendSub = a;
    dubsup = d;
    addRequirements(a);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (dubsup.getAsDouble() > 0.25 || dubsup.getAsDouble() < -0.25) {
      armExtendSub.setExtend(dubsup.getAsDouble());
    } else {
      armExtendSub.setArmExtensionPosition();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
