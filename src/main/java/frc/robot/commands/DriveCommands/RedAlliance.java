// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive.DriveTrainConstants;
import frc.robot.subsystems.Drive.DriveTrainSubsystems;

public class RedAlliance extends CommandBase {
  /** Creates a new RedAlliance. */
  public DriveTrainSubsystems driveSub;
  public PIDController pid = new PIDController(.055, 0.0, 0.01);

  public RedAlliance(DriveTrainSubsystems dSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveSub = dSub;
    addRequirements(driveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double correction = pid.calculate(driveSub.getPitch() + driveSub.getRoll(), 0);
    driveSub.fieldOrientedDriveNumber(correction, 0.0, 0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSub.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (driveSub.getPitch() + driveSub.getRoll()) < 1;
    // return false;
  }
}
