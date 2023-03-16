// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive.DriveTrainSubsystems;

// public class HomeArmCommand extends CommandBase {
//   /** Creates a new HomeArmCommand. */
//   ArmAngleSubsystem armAngleSubsytem;
//   public HomeArmCommand(ArmAngleSubsystem armSub) {
//     armAngleSubsytem = armSub;
//     addRequirements(armSub);
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     armAngleSubsytem.setPercentage(.075);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     armAngleSubsytem.setPercentage(0);
//     armAngleSubsytem.zeroPos();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     // return armAngleSubsytem.isAtLow();
//     return false;
//   }
// }
