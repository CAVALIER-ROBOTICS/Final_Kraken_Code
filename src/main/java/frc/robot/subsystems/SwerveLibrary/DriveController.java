// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SwerveLibrary;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

/** Add your docs here. */
public interface DriveController {
    TalonFX motor = null;

    void setReferenceVoltage(double voltage);

    double getEncoder();

    double getStateVelocity();
}