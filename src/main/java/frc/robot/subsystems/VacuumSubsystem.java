// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.TalonSRXControlMode; HMMM DO WE USE THIS???
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VacuumSubsystem extends SubsystemBase {
  /** Creates a new VacuumSubsystem. */
  TalonSRX vacuumMotor99 = new TalonSRX(Constants.vacuum99ID);
  TalonSRX vacuumMotor88 = new TalonSRX(Constants.vacuum88ID);
  TalonSRX vacuumMotor77 = new TalonSRX(Constants.vacuum77ID);

  public VacuumSubsystem() {}

  public void setMotorVelocity(double x) {
      vacuumMotor99.set(ControlMode.Velocity, x);
      vacuumMotor88.set(ControlMode.Velocity, x);
      vacuumMotor77.set(ControlMode.Velocity, x);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
