// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ExtendScoring;
import frc.robot.commands.VacuumCommand;
import frc.robot.commands.WristCommand;
import frc.robot.commands.ArmCommands.ArmAngleCommand;
import frc.robot.commands.ArmCommands.ArmInCommand;
import frc.robot.commands.ArmCommands.ArmOutCommand;
import frc.robot.commands.DriveCommands.ABV2Command;
import frc.robot.commands.DriveCommands.FieldDriveCommand;
import frc.robot.commands.DriveCommands.LockCommand;
import frc.robot.commands.DriveCommands.RedAlliance;
import frc.robot.subsystems.WristRotSub;
import frc.robot.subsystems.WristSub;
import frc.robot.subsystems.ArmSubsystems.ArmAngleSubsystem;
import frc.robot.subsystems.ArmSubsystems.ArmExtendSubsystem;
import frc.robot.subsystems.Drive.DriveTrainSubsystems;
import frc.robot.subsystems.VacuumSubsystems.VacuumSubsystem;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static XboxController driver = new XboxController(0);
  public static XboxController operator = new XboxController(1);
  public static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-cavbots"); // TODO move to

  // drive subsystems
  DriveTrainSubsystems driveSub = new DriveTrainSubsystems();
  DriveAuto driveAuto = new DriveAuto(driveSub);
  PathPlannerTrajectory path;

  // arm subsystems
  ArmAngleSubsystem armAngleSub = new ArmAngleSubsystem();
  ArmExtendSubsystem armExtendSub = new ArmExtendSubsystem();

  VacuumSubsystem vacSub = new VacuumSubsystem();

  WristRotSub wristRotSub = new WristRotSub();
  WristSub wristSubsystem = new WristSub();

  Trigger armIn = new Trigger(() -> getLeftTrigger(operator));
  Trigger armOut = new Trigger(() -> getRightTrigger(operator));

  public RobotContainer() {

    // camera = CameraServer.startAutomaticCapture();
    // MjpegServer server = new MjpegServer("Camera", 1181);
    // server.setFPS(30);
    // server.setCompression(0);
    // server.setSource(camera);

    HttpCamera limelightFeed = new HttpCamera("limelight-cavbot", "http://10.74.92.101:5800",
        HttpCameraKind.kMJPGStreamer);
    CameraServer.startAutomaticCapture(limelightFeed);

    driveSub.setDefaultCommand(
        new FieldDriveCommand(
            () -> modifyAxis(driver.getLeftY() *
                DriveTrainSubsystems.maxVelocityPerSecond),
            () -> modifyAxis(driver.getLeftX() *
                DriveTrainSubsystems.maxVelocityPerSecond),
            () -> modifyAxis(-driver.getRightX() *
                DriveTrainSubsystems.maxAnglarVelocityPerSecond * .75),
            driveSub));

    configureButtonBindings();
  }

  private void configureButtonBindings() {

    table.getEntry("camMode").setNumber(0);
    table.getEntry("ledMode").setNumber(0);

    // JoystickButton changeDrive = new JoystickButton(driver, 3);
    JoystickButton xLock = new JoystickButton(driver, 3);
    JoystickButton babyMode = new JoystickButton(driver, 1);
    JoystickButton resetGyro = new JoystickButton(driver, 4);

    JoystickButton alignBot = new JoystickButton(driver, 6);
    JoystickButton alignBotRightmost = new JoystickButton(driver, 5);

    JoystickButton setVacuumCone = new JoystickButton(operator, 2);
    JoystickButton setVacuumCube = new JoystickButton(operator, 1);

    JoystickButton wristCCW = new JoystickButton(operator, 5);
    JoystickButton wristCW = new JoystickButton(operator, 6);

    JoystickButton highScoring = new JoystickButton(operator, 4);
    JoystickButton middleScoring = new JoystickButton(operator, 3);

    // DRIVER BUTTONS

    alignBot.toggleOnTrue(new RunCommand(
        () -> driveSub.autoAlignDrive(driver::getLeftY, driver::getLeftX, driver::getRightX, false), driveSub));
    alignBotRightmost.onTrue(new RunCommand(
        () -> driveSub.autoAlignDrive(driver::getLeftY, driver::getLeftX, driver::getRightX, true), driveSub));

    alignBot.onFalse(new FieldDriveCommand(
        () -> modifyAxis(driver.getLeftY() *
            DriveTrainSubsystems.maxVelocityPerSecond),
        () -> modifyAxis(driver.getLeftX() *
            DriveTrainSubsystems.maxVelocityPerSecond),
        () -> modifyAxis(-driver.getRightX() *
            DriveTrainSubsystems.maxAnglarVelocityPerSecond),
        driveSub));

    alignBotRightmost.onFalse(new FieldDriveCommand(
        () -> modifyAxis(driver.getLeftY() *
            DriveTrainSubsystems.maxVelocityPerSecond),
        () -> modifyAxis(driver.getLeftX() *
            DriveTrainSubsystems.maxVelocityPerSecond),
        () -> modifyAxis(-driver.getRightX() *
            DriveTrainSubsystems.maxAnglarVelocityPerSecond),
        driveSub));

    babyMode.toggleOnTrue(new FieldDriveCommand(
        () -> modifyAxis((driver.getLeftY() *
            DriveTrainSubsystems.maxVelocityPerSecond) * .24),
        () -> modifyAxis((driver.getLeftX() *
            DriveTrainSubsystems.maxVelocityPerSecond) * .21),
        () -> modifyAxis(-(driver.getRightX() *
            DriveTrainSubsystems.maxAnglarVelocityPerSecond) * .20),
        driveSub));

    resetGyro.whileTrue(new InstantCommand(driveSub::zeroGyroscope));

    xLock.toggleOnTrue(new LockCommand(driveSub));

    // OPERTAOR BUTTONS

    // cones
    highScoring.onTrue(new ExtendScoring(armExtendSub, 1));
    middleScoring.onTrue(new ExtendScoring(armExtendSub, 2));

    armAngleSub.setDefaultCommand(new SequentialCommandGroup(
        // new HomeArmCommand(armAngleSub),
        new ArmAngleCommand(armAngleSub, operator::getLeftY)));
    // operator::getRightY));

    setVacuumCone.toggleOnTrue(new VacuumCommand(vacSub, 1));
    setVacuumCube.toggleOnFalse(new VacuumCommand(vacSub, -1));

    armOut.whileTrue(new ArmOutCommand(armExtendSub));
    armIn.whileTrue(new ArmInCommand(armExtendSub));

    wristSubsystem.setDefaultCommand(new WristCommand(wristSubsystem, operator::getRightY));

    wristCW.whileTrue(new StartEndCommand(
        () -> wristRotSub.setWrist(.4),
        () -> wristRotSub.setWrist(0),
        wristRotSub));

    wristCCW.whileTrue(new StartEndCommand(
        () -> wristRotSub.setWrist(-.4),
        () -> wristRotSub.setWrist(0),
        wristRotSub));

    // changeDrive.toggleOnTrue(
    // new FieldDriveCommand(
    // () -> modifyAxis(driver.getRawAxis(1)),
    // () -> modifyAxis(driver.getRawAxis(0)),
    // () -> modifyAxis(driver.getRawAxis(4)),
    // driveSub));

    // driveForward.whileTrue(new InstantCommand( () -> driveSub.drive(new
    // ChassisSpeeds(0, 10, 0))));
  }

  // public void resetOdo() {
  // driveSub.resetOdometry(path.getInitialPose());
  // }

  // public void updateOdometry() {
  // driveSub.updateOdo();
  // }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.1) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.08);
    SmartDashboard.putNumber("wheelSpeedinput", value);

    // value = Math.copySign(value * value, value);
    return value;
  }

  public DriveTrainSubsystems getDriveSub() {
    return driveSub;
  }

  private static boolean getRightTrigger(XboxController controller) {
    return controller.getRightTriggerAxis() > 0.05;
  }

  public Command getDriveCommand() {
    return new SequentialCommandGroup(
        new InstantCommand(driveSub::zeroGyroscope),
        new RunCommand(() -> driveSub.fieldOrientedDriveNumber(-1.0, 0.0, 0.0), driveSub).withTimeout(1),
        new InstantCommand(() -> driveSub.drive(new ChassisSpeeds(0, 0, 0))),
        new RedAlliance(driveSub), //Swap to ABV2Command if charge station is good (blue)
        new LockCommand(driveSub)
    );
  }

  public Command getDriveArmCommand() {
    return new SequentialCommandGroup(
        // new RunCommand(() -> armAngleSub.setAnglePosition(30)).withTimeout(1),

        // new InstantCommand(() -> armAngleSub.setPercentage(0)),

        new InstantCommand(driveSub::zeroGyroscope),

        // backward facing
        new RunCommand(() -> driveSub.fieldOrientedDriveNumber(1.0, 0.0, 0.0), driveSub).withTimeout(2.5),

        new InstantCommand(() -> driveSub.drive(new ChassisSpeeds(0, 0, 0))));
  }

  public Command getAutoDrive() {
    return new SequentialCommandGroup(
        driveAuto.getPath("Devin2"));
  }

  public Command getBalance() {
    return new ABV2Command(driveSub);
  }

  private static boolean getShouldNegate(XboxController controller) {
    return controller.getRightTriggerAxis() < controller.getLeftTriggerAxis();
  }

  private static boolean getLeftTrigger(XboxController controller) {
    return controller.getLeftTriggerAxis() > 0.05;
  }

  private static boolean getOperatorLeftBumper() {
    return driver.getRawButton(3);
  }

  private static boolean getOperatorRightBumper() {
    return driver.getRawButton(1);
  }
}