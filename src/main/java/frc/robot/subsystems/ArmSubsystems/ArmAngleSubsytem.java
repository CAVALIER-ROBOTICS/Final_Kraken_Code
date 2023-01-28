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
  public CANSparkMax angleMotor = new CANSparkMax(Constants.armMoveID, MotorType.kBrushless);
  private RelativeEncoder angleEncoder = angleMotor.getEncoder();
  private SparkMaxPIDController anglePID = angleMotor.getPIDController();
  
  public ArmAngleSubsytem() {
    angleMotor.restoreFactoryDefaults();
    angleMotor.setIdleMode(IdleMode.kBrake);
    angleMotor.setInverted(true); //I believe it should be inverted??

    anglePID.setP(0.05);
    anglePID.setI(0.01);
    anglePID.setD(0.001);

    angleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
    angleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);
  }

  public void setAngle(double x) {
    angleMotor.setVoltage(x);
  }

  public double getVoltage() {
    return angleMotor.getOutputCurrent();
  }

  public void setAnglePosition() {
    anglePID.setReference(angleEncoder.getPosition(), CANSparkMax.ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Angle Output:", getVoltage());
  }
}
