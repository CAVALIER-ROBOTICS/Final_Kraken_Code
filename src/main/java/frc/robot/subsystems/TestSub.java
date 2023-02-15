// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestSub extends SubsystemBase {
  CANSparkMax motor = new CANSparkMax(0, MotorType.kBrushless);
  RelativeEncoder enc = motor.getEncoder();
  /** Creates a new TestSub. */
  public TestSub() {}

  public void setMotorVoltage(double volts) {
    motor.setVoltage(volts);
  }

  public double getVelocity() {
    return enc.getVelocity();
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
}
