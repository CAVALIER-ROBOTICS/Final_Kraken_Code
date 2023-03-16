// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive.DriveTrainSubsystems;
import frc.robot.subsystems.Drive.Trajectories.TrajectoriesSubsystem;

public class AutoBalanceCommand extends CommandBase {
  private double balanaceEffort; // The effort the robot should use to balance
  private double turningEffort; // The effort the robot should use to turn

  public TrajectoriesSubsystem trajectories;

  public DriveTrainSubsystems driveSub;

  /*Creates a new GeneratePath.
   * * @param firstPoints*/
  public AutoBalanceCommand(DriveTrainSubsystems dSub) {
      // Use addRequirements() here to declare subsystem dependencies.
      driveSub = dSub;
      addRequirements(driveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      turningEffort =
              trajectories.calculateThetaSupplier(() -> Constants.AutoBalanceConfig.angleSetPoint).getAsDouble();
      balanaceEffort =
              (Constants.AutoBalanceConfig.balancedAngle - Math.toDegrees(driveSub.getRoll()))
                      * Constants.AutoBalanceConfig.kP;
      // driveSub.drive(balanaceEffort, 0, turningEffort, false, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      driveSub.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return Math.abs(Math.toDegrees(driveSub.getRoll())) < 2;
  }
}