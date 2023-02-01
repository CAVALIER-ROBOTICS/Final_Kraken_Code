// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystems;

public class ABV2Command extends CommandBase {
  PIDController pitchController = new PIDController(.07, 0, 0);
  PIDController rollController = new PIDController(.07, 0, 0);
  DriveTrainSubsystems dSub;

  /** Creates a new ABV2Command. */
  public ABV2Command(DriveTrainSubsystems d) {
    dSub = d;
    addRequirements(dSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pitchController.enableContinuousInput(-30, 30);
    rollController.enableContinuousInput(-30, 30);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pitchCorrection = pitchController.calculate(dSub.getPitch());
    double rollCorrection = rollController.calculate(dSub.getRoll());

    SmartDashboard.putString("RollAndPitch", String.valueOf(-pitchCorrection) + ", " + String.valueOf(-rollCorrection));
    dSub.driveFromNumRobotOriented(-pitchCorrection, -rollCorrection, 0.0);

    if(Math.abs(pitchCorrection) + Math.abs(rollCorrection) < 1) {
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}