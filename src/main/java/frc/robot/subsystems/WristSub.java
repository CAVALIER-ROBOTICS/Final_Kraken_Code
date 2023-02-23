// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSub extends SubsystemBase {
  CANSparkMax wristMotor = new CANSparkMax(44, MotorType.kBrushless);
  SparkMaxPIDController wristPID = wristMotor.getPIDController();
  RelativeEncoder wristEncoder = wristMotor.getEncoder();
  /** Creates a new WristSub. */
  public WristSub() {
    wristPID.setP(0.0);
    wristPID.setI(0.0);
    wristPID.setD(0.0);
  }

  public void setWrist(double percentage) {
    SmartDashboard.putNumber("Wrist set lol", percentage);
    wristMotor.set(percentage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
