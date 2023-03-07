// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Field;
import java.util.NoSuchElementException;

import javax.lang.model.util.ElementScanner14;

import org.opencv.aruco.Aruco;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveTrainSubsystems;
import frc.robot.subsystems.ArmSubsystems.ArmAngleSubsytem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command autoBalance;
  private Command drive;

  private RobotContainer robotContainer;
  // private final I2C.Port i2cport = I2C.Port.kOnboard;
  // ColorSensorV3 colorSensorV3 = new ColorSensorV3(i2cport);
  // private final ColorMatch cMatch = new ColorMatch();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
    drive = robotContainer.getDriveCommand();

    // cMatch.addColorMatch(kBlueTarget);
    // cMatch.addColorMatch(kRedTarget);

    // cMatch.setConfidenceThreshold(0.96);
    // PortForwarder.add(5800, "limelight.local", 5800);
    // PortForwarder.add(5801, "limelight.local", 5801);
    // PortForwarder.add(5802, "limelight.local", 5802);
    // PortForwarder.add(5803, "limelight.local", 5803);
    // PortForwarder.add(5804, "limelight.local", 5804);
    // PortForwarder.add(5805, "limelight.local", 5805);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    // robotContainer.updateOdometry();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    ArmAngleSubsytem armAngleSubsytem = robotContainer.getArmAngleSub();
    armAngleSubsytem.stopArm();

    DriveTrainSubsystems driveSub = robotContainer.getDriveSub();
    driveSub.drive(new ChassisSpeeds(1, 0, 0));
  }

  @Override
  public void disabledPeriodic() {
  
    // DriveTrainSubsystems driveSub = robotContainer.getDriveSub();
    // driveSub.drive(new ChassisSpeeds(0, 0, 0));
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // robotContainer.resetOdo();

    // if (autoBalance != null) {
    // autoBalance.schedule();
    // }

    if (drive != null) {
      drive.schedule();
    }

  

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    if (autoBalance != null) {
      autoBalance.cancel();
    }

    if (drive != null) {
      drive.cancel();
    }

    // if(autoKick != null) {
    // autoKick.cancel();
    // }

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Color detectedColor = colorSensorV3.getColor();
    // String cString;
    // ColorMatchResult matchResult = cMatch.matchClosestColor(detPectedColor);
    // int proxy = colorSensorV3.getProximity();

    // if((matchResult.color == kBlueTarget)&&(detectedColor.blue>.3)) {
    // cString = "Blue";
    // }
    // else if((matchResult.color == kRedTarget)&&(detectedColor.red>.4)) {
    // cString = "Red";
    // }
    // else {
    // cString = "Undefined";
    // }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
