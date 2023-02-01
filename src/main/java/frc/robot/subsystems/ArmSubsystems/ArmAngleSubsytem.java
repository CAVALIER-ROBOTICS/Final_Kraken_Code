// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ArmSubsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

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
  private SparkMaxPIDController anglePIDRight = angleMotorRight.getPIDController();
  
  public ArmAngleSubsytem() {
    angleMotorLeft.restoreFactoryDefaults();
    angleMotorLeft.setIdleMode(IdleMode.kBrake);
    angleMotorLeft.setInverted(true); //I believe it should be inverted??

    angleMotorRight.restoreFactoryDefaults();
    angleMotorRight.setIdleMode(IdleMode.kBrake);
    angleMotorRight.setInverted(true); //I believe it should be inverted??

    anglePIDLeft.setP(0.05);
    anglePIDLeft.setI(0.01);
    anglePIDLeft.setD(0.001);

    anglePIDRight.setP(0.05);
    anglePIDRight.setI(0.01);
    anglePIDRight.setD(0.001);

    angleMotorLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
    angleMotorLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);

    angleMotorRight.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
    angleMotorRight.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);
  }

  public void setAngle(double x) {
    angleMotorLeft.setVoltage(x);
    angleMotorRight.setVoltage(x);
  }

  //average of both
  public double getVoltage() {
    return ( angleMotorLeft.getOutputCurrent() + angleMotorRight.getOutputCurrent() ) / 2;
  }

  public void setAnglePosition() {
    anglePIDLeft.setReference(angleEncoderLeft.getPosition(), CANSparkMax.ControlType.kPosition);
    anglePIDRight.setReference(angleEncoderLeft.getPosition(), CANSparkMax.ControlType.kPosition);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Angle Output:", getVoltage());
  }
}
