// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SetPoints;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.ArmCommands.ArmAngleCommand;
import frc.robot.subsystems.ArmSubsystems.ArmAngleSubsystem;

// import frc.robot.subsystems.ArmSubsystems.ArmarmSub;

public class ArmCommand extends CommandBase {
  /** Creates a new armCommand. */
  ArmAngleSubsystem armSub;
  Timer timer;
  public ArmCommand(ArmAngleSubsystem aSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    armSub = aSub;
    addRequirements(aSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSub.zeroPos();
    armSub.setPercentage(.25);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSub.setPercentage(0);
    armSub.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (armSub.getAngle() >= .25) {
      return true;
    } else {
      return false;
    }
  }
}
