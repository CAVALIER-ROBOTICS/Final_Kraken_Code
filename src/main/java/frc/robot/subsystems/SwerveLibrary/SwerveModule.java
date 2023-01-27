// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SwerveLibrary;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.kinematics.SwerveModulePosition;

/** Add your docs here. */
public interface SwerveModule {

    double getDriveVelocity();

    double getSteerAngle();

    SwerveModulePosition getPosition();

    void set(double driveVoltage, double steerAngle);
}