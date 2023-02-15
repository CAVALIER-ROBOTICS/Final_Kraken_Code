// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TestSub;

public class NEOTestCommand extends CommandBase {
  Timer time = new Timer();
  TestSub testSub;
  double maxVelocity = 0;
  /** Creates a new NEOTestCommand. */
  public NEOTestCommand(TestSub sub) {
    testSub = sub;
    addRequirements(testSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.start();
    testSub.setMotorVoltage(12);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double velocity = testSub.getVelocity();
    if(velocity > maxVelocity) {
      maxVelocity = velocity;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putNumber("NEO Max velocity result", maxVelocity);
    time.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return time.get() > 10;
  }
}
