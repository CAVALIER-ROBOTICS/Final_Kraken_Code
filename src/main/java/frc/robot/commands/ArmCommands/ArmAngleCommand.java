// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystems.ArmAngleSubsytem;

public class ArmAngleCommand extends CommandBase {
  /** Creates a new ArmAngleCommand. */
  ArmAngleSubsytem armAngleSub;
  DoubleSupplier dubsup;

  public ArmAngleCommand(ArmAngleSubsytem a, DoubleSupplier d) {
    // Use addRequirements() here to declare subsystem dependencies.
    armAngleSub = a;
    dubsup = d;
    addRequirements(a);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (dubsup.getAsDouble() > 0.25 || dubsup.getAsDouble() < -0.25) {
      armAngleSub.setAngle(dubsup.getAsDouble());
    } else {
      armAngleSub.setAnglePosition();
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
