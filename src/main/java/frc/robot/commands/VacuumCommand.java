// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VacuumSubsystem;

public class VacuumCommand extends CommandBase {
  /** Creates a new VacuumCommand. */
  VacuumSubsystem vacuumSub;

  public VacuumCommand(VacuumSubsystem v) {
    // Use addRequirements() here to declare subsystem dependencies.
    vacuumSub = v;
    addRequirements(v);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    vacuumSub.setMotorVelocity(15000); //can change with how much suction neccessary
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    vacuumSub.setMotorVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
