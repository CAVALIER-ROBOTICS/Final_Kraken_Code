// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ArmSubsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmAngleSubsytem extends SubsystemBase {
  /** Creates a new ArmAngle. */
  public CANSparkMax angleMotorLeft = new CANSparkMax(Constants.armAngle11ID, MotorType.kBrushless);
  public CANSparkMax angleMotorRight = new CANSparkMax(Constants.armAngle22ID, MotorType.kBrushless);

  private RelativeEncoder angleEncoderLeft = angleMotorLeft.getEncoder();
  private RelativeEncoder angleEncoderRight = angleMotorRight.getEncoder();

  private SparkMaxPIDController anglePIDLeft = angleMotorLeft.getPIDController();

  DigitalInput limitSwitch = new DigitalInput(9);

  public ArmAngleSubsytem() {
    angleMotorLeft.restoreFactoryDefaults();
    angleMotorRight.restoreFactoryDefaults();

    angleMotorRight.follow(angleMotorLeft, true);
    angleMotorRight.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
    angleMotorLeft.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
  }

  public void setVoltage(double x) {
    angleMotorLeft.setVoltage(x);
    // angleMotorRight.setVoltage(x);
  }

  // average of both
  public double getVoltage() {
    return (angleMotorLeft.getOutputCurrent() + angleMotorRight.getOutputCurrent()) / 2;
  }

  public void setAnglePosition(double setpoint) {
    anglePIDLeft.setP(.02);
    anglePIDLeft.setI(0.000001);
    anglePIDLeft.setD(0);
    anglePIDLeft.setFF(.0004);
    
    double encoderRot = (setpoint + 10) / Constants.ARM_MAX * 50;
    anglePIDLeft.setReference(encoderRot, CANSparkMax.ControlType.kPosition);
    // anglePIDRight.setReference(encoderRot, CANSparkMax.ControlType.kPosition);

  }

  public void setPercentage(double percent) {
    angleMotorLeft.set(percent);
    // angleMotorRight.set(-percent);
  }

  public void stopArm() {
    anglePIDLeft.setP(.3);
    anglePIDLeft.setI(.01);
    anglePIDLeft.setD(.075);
    anglePIDLeft.setFF(0);

    anglePIDLeft.setReference(angleEncoderLeft.getPosition(), CANSparkMax.ControlType.kPosition);
  }

  public double getAngle() {
    return angleEncoderLeft.getPosition() * 360 / 50;
  }

  public void resetEncoder() {
    angleEncoderLeft.setPosition(0.0);
  }

  public void stopLoop() {
    angleMotorLeft.set(0.0);
  }

  public boolean isAtLow() {
    return limitSwitch.get();
  }

  public void zeroPos() {
    angleEncoderLeft.setPosition(0.0);
    angleEncoderRight.setPosition(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Angle Output:", getVoltage());
    SmartDashboard.putNumber("Encoder pos", angleEncoderLeft.getPosition());
  }
}
