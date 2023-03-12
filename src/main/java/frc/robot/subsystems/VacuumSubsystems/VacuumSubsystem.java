// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.VacuumSubsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VacuumSubsystem extends SubsystemBase {
  // left
  TalonSRX m1 = new TalonSRX(32);
  // right
  TalonSRX m2 = new TalonSRX(60);
  TalonSRX m3 = new TalonSRX(62);

  public static enum motors {
    M1,
    M2,
    M3
  }

  /** Creates a new VacuumSubsystem. */
  public VacuumSubsystem() {
    m1.enableVoltageCompensation(true);
    m2.enableVoltageCompensation(true);
  }

  public void setAll(double percentage) {
    m1.set(ControlMode.PercentOutput, percentage);
    m2.set(ControlMode.PercentOutput, percentage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
