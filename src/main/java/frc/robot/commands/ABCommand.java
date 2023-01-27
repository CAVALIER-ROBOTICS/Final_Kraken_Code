// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveTrainSubsystems;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ABCommand extends PIDCommand {
  /** Creates a new ABCommand. */
  public ABCommand(DriveTrainSubsystems driveSub) {
    super(
        // The controller that the command will use
        new PIDController(.09, 0, 0),
        // This should return the measurement
        driveSub::getPitch,
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          SmartDashboard.putNumber("BalanceOutput", -output);
          driveSub.driveFromNum(-output, 0.0, 0.0);
        });
    addRequirements(driveSub);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
