// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristRotSub extends SubsystemBase {
  CANSparkMax wristRotMotor = new CANSparkMax(Constants.wristRotID, MotorType.kBrushless);
  /** Creates a new WristRotSub. */
  public WristRotSub() {}

  public void setWrist(double setpoint) {
    wristRotMotor.set(setpoint);
    wristRotMotor.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
