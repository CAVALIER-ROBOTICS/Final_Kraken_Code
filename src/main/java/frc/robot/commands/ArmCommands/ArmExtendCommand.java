// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystems.ArmExtendSubsystem;

public class ArmExtendCommand extends CommandBase {
  /** Creates a new ArmAngleCommand. */
  ArmExtendSubsystem armExtendSub;
  BooleanSupplier dubsup;

  public ArmExtendCommand(ArmExtendSubsystem a, BooleanSupplier d) {
    // Use addRequirements() here to declare subsystem dependencies.
    armExtendSub = a;
    dubsup = d;
    addRequirements(a);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!dubsup.getAsBoolean()) {
      armExtendSub.setExtend(.4);
    } else {
      armExtendSub.setExtend(-.4);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armExtendSub.setExtend(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Boolean shouldNegate = dubsup.getAsBoolean();
    if(!shouldNegate) {
      return armExtendSub.getIsAtHigh();
    } else {
      return armExtendSub.getIsAtLow();
    }
  }
}
