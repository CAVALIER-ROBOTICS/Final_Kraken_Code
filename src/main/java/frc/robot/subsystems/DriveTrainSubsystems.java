// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Liberderry.MechanicalConfiguration;
import frc.robot.subsystems.Liberderry.MkSwerveModuleBuilder;
import frc.robot.subsystems.Liberderry.MotorType;
import frc.robot.subsystems.Liberderry.SdsModuleConfigurations;
import frc.robot.subsystems.Liberderry.SwerveModule;
import frc.robot.swerve.TorqueSwerveModule2022;
import frc.robot.swerve.TorqueSwerveModule2022.SwerveConfig;
import frc.robot.swerve.base.TorqueSwerveModule;

import java.util.concurrent.TimeUnit;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrainSubsystems extends SubsystemBase implements DriveTrainConstants {
  /** Creates a new DriveTrainSubsystems. */

  // search up Pigeon IMU for more info
  WPI_Pigeon2 pidgey = new WPI_Pigeon2(pigeonID, "OTHERCANIVORE");
  // Odometry class for tracking robot pose
  // Rotation2d.fromDegrees(getFusedHeading()); getRotation2d()
  // Rotation2d.fromDegrees(pidgey.getCompassHeading()
  private final SwerveDriveOdometry odo;

  // These are the modules initialize them in the constructor.
  private final TorqueSwerveModule2022 frontLeftModule;
  private final TorqueSwerveModule2022 frontRightModule;
  private final TorqueSwerveModule2022 backLeftModule;
  private final TorqueSwerveModule2022 backRightModule;

  // @warning may not be right, just to make code work for now
  public static final double maxVelocityPerSecond = 6380.0 / 60.0 *
      SdsModuleConfigurations.MK4_L2.getDriveReduction() *
      SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

  public static final double maxAnglarVelocityPerSecond = maxVelocityPerSecond /
      Math.hypot(Constants.W / 2.0, Constants.L / 2.0);

  private double xVelocity;
  private double yVelocity;

  public DriveTrainSubsystems() {

    final SwerveConfig config = SwerveConfig.defaultConfig;

    backLeftModule = new TorqueSwerveModule2022("Back Left", DriveTrainConstants.backLeftDriveMotor,
        DriveTrainConstants.backLeftSteerMotor, DriveTrainConstants.backLeftSteerEncoder,
        DriveTrainConstants.backLeftModuleSteerOffset, config);

    backRightModule = new TorqueSwerveModule2022("Back Right", DriveTrainConstants.backRightDriveMotor,
        DriveTrainConstants.backRightSteerMotor, DriveTrainConstants.backRightSteerEncoder,
        DriveTrainConstants.backRightModuleSteerOffset, config);

    frontLeftModule = new TorqueSwerveModule2022("Front Left", DriveTrainConstants.frontLeftDriveMotor,
        DriveTrainConstants.frontLeftSteerMotor, DriveTrainConstants.frontLeftSteerEncoder,
        DriveTrainConstants.frontLeftModuleSteerOffset, config);

    frontRightModule = new TorqueSwerveModule2022("Front Right", DriveTrainConstants.frontRightDriveMotor,
        DriveTrainConstants.frontRightSteerMotor, DriveTrainConstants.frontRightSteerEncoder,
        DriveTrainConstants.frontRightModuleSteerOffset, config);
    // pidgey.setStatusFramePeriod(PigeonIMU_StatusFrame.RawStatus_4_Mag, 62200);
    // pidgey.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_6_Accel,
    // 62100);
    // pidgey.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_2_Gyro,
    // 62150);

    // updatePositions();
    odo = new SwerveDriveOdometry(Constants.m_kinematics, pidgey.getRotation2d(), getModulePositions());

  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] { frontLeftModule.getPosition(), frontRightModule.getPosition(),
        backRightModule.getPosition(), backLeftModule.getPosition() };
  }

  public SwerveModulePosition getPos(SwerveModule state) {
    return state.getPosition();
  }

  public void zeroGyroscope() {
    pidgey.reset();
  }

  public Rotation2d getGyroscopeRotation() {
    // We have to invert the angle of the NavX so that rotating the robot
    // counter-clockwise makes the angle increase.
    return Rotation2d.fromDegrees(-pidgey.getAngle() % 360);
  }

  public void drive(ChassisSpeeds speeds) {
    SwerveModuleState[] states = Constants.m_kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, 3);

    frontLeftModule.setDesiredState(states[0]);
    frontRightModule.setDesiredState(states[1]);
    backRightModule.setDesiredState(states[2]);
    backLeftModule.setDesiredState(states[3]);
    SmartDashboard.putNumber("pigeon rotation :)", pidgey.getRotation2d().getDegrees());
  }

  @Override
  public void periodic() {
  }

  public void updateOdo() {
    odo.update(pidgey.getRotation2d(), getModulePositions());

    // SmartDashboard.putString("Odo", ""+odo.getPoseMeters());
  }

  public Pose2d getPose() {
    return odo.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odo.resetPosition(pose.getRotation(), getModulePositions(), pose);
  }

  public void fieldOrientedDrive(DoubleSupplier xtrans, DoubleSupplier ytrans, DoubleSupplier rot) {
    drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xtrans.getAsDouble(),
            ytrans.getAsDouble(),
            rot.getAsDouble(),
            getGyroscopeRotation()));
    xVelocity = xtrans.getAsDouble();
    yVelocity = ytrans.getAsDouble();
  }

  public void robotOrientedDrive(double xtrans, double ytrans, double rot) {
    ChassisSpeeds speeds = new ChassisSpeeds(
        xtrans,
        ytrans,
        rot);

    drive(speeds);
  }

  public void driveFromNum(Double xtrans, Double ytrans, Double rot) {
    drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xtrans,
            ytrans,
            rot,
            getGyroscopeRotation()));
    xVelocity = xtrans;
    yVelocity = ytrans;
  }

  public void driveFromNumRobotOriented(Double xtrans, Double ytrans, Double rot) {
    drive(
        new ChassisSpeeds(
            xtrans,
            ytrans,
            rot));
    xVelocity = xtrans;
    yVelocity = ytrans;
  }

  public void stop() {
    driveFromNum(0.0, 0.0, 0.0);
  }

  public double getSpeed() {
    double speed = Math.sqrt(Math.pow(2, xVelocity) + Math.pow(2, yVelocity));
    return speed;
  }

  public double getRoll() {
    double roll = pidgey.getRoll();
    SmartDashboard.putNumber("Roll", roll);
    return roll;
  }

  public double getPitch() {
    double pitch = pidgey.getPitch();
    SmartDashboard.putNumber("Pitch", pitch);
    return pitch;
  }

  public double getYaw() {
    return pidgey.getYaw();
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] { frontLeftModule.getState(), frontRightModule.getState(),
        backRightModule.getState(), backLeftModule.getState() };
  }
}
