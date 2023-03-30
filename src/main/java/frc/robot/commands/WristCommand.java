// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSub;

public class WristCommand extends CommandBase {
  /** Creates a new WristCommand. */
  WristSub wristSubsystem;
  DoubleSupplier dubSubLol;
  public WristCommand(WristSub wSub, DoubleSupplier dSup) {
    wristSubsystem = wSub;
    addRequirements(wristSubsystem);
    dubSubLol = dSup;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double input = dubSubLol.getAsDouble();

    if(Math.abs(input) > .3) {
      wristSubsystem.setWrist(dubSubLol.getAsDouble() * -.3);
    } else {
      wristSubsystem.stopWrist();
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
