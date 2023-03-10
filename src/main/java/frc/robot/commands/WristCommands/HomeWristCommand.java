// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.WristCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSub;

public class HomeWristCommand extends CommandBase {
  /** Creates a new HomeWristCommand. */
  WristSub wSub = new WristSub();
  public HomeWristCommand(WristSub w) {
    wSub = w;
    addRequirements(w);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wSub.setWrist(.25);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wSub.setWrist(0);
    wSub.zeroWrist();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return wSub.getWristUp();
  }
}
