// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristSub extends SubsystemBase {
  CANSparkMax wristMotor = new CANSparkMax(44, MotorType.kBrushless);
  SparkMaxPIDController wristPID = wristMotor.getPIDController();
  RelativeEncoder wristEncoder = wristMotor.getEncoder();

  DigitalInput limitSwitch = new DigitalInput(6);

  /** Creates a new WristSub. */
  public WristSub() {
    wristPID.setP(0.001);
    wristPID.setI(0.0025);
    wristPID.setD(0.0);

    wristMotor.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
    wristEncoder.setPosition(0.0);
  }

  public void setAngle(double angle) {
    double encoderPos = angle / Constants.WRIST_MAX * 50;
    wristPID.setReference(encoderPos, ControlType.kPosition);
  }

  public double getAngle() {
    return wristEncoder.getPosition() * Constants.WRIST_MAX / 50;
  }

  public void setWrist(double percentage) {
    wristMotor.set(percentage);
  }

  public void stopWrist() {
    setAngle(getAngle());
  }

  public void zeroWrist() {
    wristEncoder.setPosition(0.0);
  }

  public boolean getWristUp() {
    return limitSwitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
