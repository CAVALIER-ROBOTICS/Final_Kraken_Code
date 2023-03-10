// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystems.ArmExtendSubsystem;

public class AutoArmCommand extends CommandBase {
  /** Creates a new AutoArmCommand. */
  ArmExtendSubsystem armExtendSub = new ArmExtendSubsystem();
  public AutoArmCommand(ArmExtendSubsystem a) {
    armExtendSub = a;
    addRequirements(a);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armExtendSub.setPercentage(.4);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armExtendSub.setPercentage(.4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armExtendSub.setPercentage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
