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

  }

  public void setAll(double percentage) {
    m1.set(ControlMode.PercentOutput, percentage);
    m2.set(ControlMode.PercentOutput, percentage);
    // m3.set(ControlMode.PercentOutput, percentage);
  }

  public void set(double percentage, motors motorType) {
    if (motorType == motors.M1) {
      m1.set(ControlMode.PercentOutput, percentage);
    }
    if (motorType == motors.M2) {
      m2.set(ControlMode.PercentOutput, percentage);
    }
    // if (motorType == motors.M3) {
    // m3.set(ControlMode.PercentOutput, percentage);
    // }
  }

  public double getMotorCurrent(motors motorType) {
    if (motorType == motors.M1) {
    return m1.getOutputCurrent();
    }
    if (motorType == motors.M2) {
      return m2.getOutputCurrent();
    }
    // if (motorType == motors.M3) {
    //   return m3.getOutputCurrent();
    // }
    return 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
