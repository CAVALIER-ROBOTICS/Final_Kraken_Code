// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystems.ArmAngleSubsytem;

public class HoldArmExtendCommand extends CommandBase {
  /** Creates a new HoldArmExtendCommand. */
  ArmAngleSubsytem armAngleSub;

  public HoldArmExtendCommand(ArmAngleSubsytem a) {
    // Use addRequirements() here to declare subsystem dependencies.
    armAngleSub = a;
    addRequirements(a);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armAngleSub.setAnglePosition();
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
