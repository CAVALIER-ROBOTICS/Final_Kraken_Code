// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase {
  /** Creates a new WristSubsystem. */
  CANSparkMax wristMotor = new CANSparkMax(Constants.wristID, MotorType.kBrushless);
  PIDController wristController = new PIDController(0, 0, 0);
  RelativeEncoder wristEncoder = wristMotor.getEncoder();

  public WristSubsystem() {
    wristMotor.restoreFactoryDefaults();
    wristMotor.setInverted(true); // I think that is what it should be??

    wristEncoder.setPosition(0);
  }

  public void setWrist(double x) {
    wristMotor.set(x);
  }

  public void setWristVoltage(double x) {
    wristMotor.setVoltage(x);
  }

  public double getWristVoltage() {
    return wristMotor.getOutputCurrent();
  }

  public void setEncoder(double x) {
    wristEncoder.setPosition(x);
  }

  public double getRadianPos() {
    return wristEncoder.getPosition() * 2 * Math.PI;
  }

  public void setWristAngle(double angle) {
    double radianSetpoint = Math.toRadians(angle);

    wristMotor.set(wristController.calculate(getRadianPos(), radianSetpoint));
  }

  public double getPosition() {
    return wristEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist Encoder:", getPosition());
  }
}
