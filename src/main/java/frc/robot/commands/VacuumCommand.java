// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.VacuumSubsystems.VacuumSubsystem;
import frc.robot.subsystems.VacuumSubsystems.VacuumSubsystem.motors;

public class VacuumCommand extends CommandBase {
  VacuumSubsystem vacSub;
  Double setpoint = .4;
  Timer timer = new Timer();

  /** Creates a new VacuumCommand. */
  public VacuumCommand(VacuumSubsystem vSub) {
    vacSub = vSub;
    addRequirements(vacSub);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.operator.setRumble(RumbleType.kBothRumble, 1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    vacSub.setAll(.6);
  }

  // Called once the command ends or is interrupted.
  @Override

  public void end(boolean interrupted) {
    vacSub.setAll(0.0);
    RobotContainer.operator.setRumble(RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
