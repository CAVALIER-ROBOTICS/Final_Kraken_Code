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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmExtendSubsystem extends SubsystemBase {
  /** Creates a new ArmExtend. */
  public CANSparkMax extendMotor = new CANSparkMax(Constants.armExtendID, MotorType.kBrushless);
  private RelativeEncoder extendEncoder = extendMotor.getEncoder();
  private SparkMaxPIDController extendPID = extendMotor.getPIDController();
  DigitalInput highExtremeDigitalInput = new DigitalInput(8);
  DigitalInput lowExtremeDigitalInput = new DigitalInput(9);
  
  public ArmExtendSubsystem() {
    extendMotor.restoreFactoryDefaults();
    extendMotor.setIdleMode(IdleMode.kBrake);
    extendMotor.setInverted(true); //I believe it should be inverted??

    extendPID.setP(0.05);
    extendPID.setI(0.01);
    extendPID.setD(0.001);
    
    extendMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
    extendMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);

    extendMotor.setOpenLoopRampRate(0.25); //.25 seconds till full throttle of extension speed
    extendMotor.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
  }

  public void setExtend(double x) {
    extendMotor.setVoltage(x);
  }

  public void setPercentage(double setpoint) {
    extendMotor.set(setpoint);
  }

  public void setArmExtensionPosition() {
    extendPID.setReference(extendEncoder.getPosition(), ControlType.kPosition);
  }
  public boolean getIsAtHigh() {
    return highExtremeDigitalInput.get();
  }

  public boolean getIsAtLow() {
    return lowExtremeDigitalInput.get();
  }

  public void zeroPos() {
    extendEncoder.setPosition(0.0);
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
