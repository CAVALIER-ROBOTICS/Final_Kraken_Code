// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystems;

public class ABCommand extends CommandBase {
  PIDController yawController = new PIDController(.3, 0, 0);

  /** Creates a new ABCommand. */
  DriveTrainSubsystems driveSub;

  public ABCommand(DriveTrainSubsystems dSub) {
    driveSub = dSub;
    addRequirements(driveSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSub.zeroGyroscope();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double correction = yawController.calculate((driveSub.getRoll()), 0);
    driveSub.drive(new ChassisSpeeds(correction, 0, 0));

    SmartDashboard.putNumber("Pitch", driveSub.getRoll());
    SmartDashboard.putNumber("Correction", correction);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSub.drive(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
