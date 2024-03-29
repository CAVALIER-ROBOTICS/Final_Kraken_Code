// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystems.ArmExtendSubsystem;

public class ArmOutCommand extends CommandBase {
  /** Creates a new ArmInCommand. */
  ArmExtendSubsystem armExtendSubsystem;

  public ArmOutCommand(ArmExtendSubsystem armSub) {
    armExtendSubsystem = armSub;
    addRequirements(armExtendSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armExtendSubsystem.setPercentage(.4);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armExtendSubsystem.setPercentage(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Low = all the way out :)
    // return armExtendSubsystem.getIsAtLow();
    return false;
  }
}
