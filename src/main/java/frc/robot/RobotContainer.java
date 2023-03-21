// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CameraServerJNI;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.ErrorMessages;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.VacuumCommand;
import frc.robot.commands.ArmCommands.ArmAngleCommand;
import frc.robot.commands.ArmCommands.ArmInCommand;
import frc.robot.commands.ArmCommands.ArmOutCommand;
import frc.robot.commands.ArmCommands.AutoArmCommand;
// import frc.robot.commands.ArmCommands.HomeArmCommand;
import frc.robot.commands.DriveCommands.ABCommand;
import frc.robot.commands.DriveCommands.FieldDriveCommand;
import frc.robot.commands.DriveCommands.RobotDriveCommand;
import frc.robot.commands.DriveCommands.TestMotorCommand;
import frc.robot.commands.SetPoints.ArmCommand;
import frc.robot.commands.SetPoints.ExtendCommand;
import frc.robot.commands.WristCommands.WristCommand;
import frc.robot.subsystems.TestMotorSub;
import frc.robot.subsystems.WristRotSub;
import frc.robot.subsystems.WristSub;
import frc.robot.subsystems.ArmSubsystems.ArmAngleSubsystem;
import frc.robot.subsystems.ArmSubsystems.ArmExtendSubsystem;
import frc.robot.subsystems.Drive.DriveTrainSubsystems;
import frc.robot.subsystems.VacuumSubsystems.VacuumSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static XboxController driver = new XboxController(0);
  public static XboxController operator = new XboxController(1);

  UsbCamera camera;

  DriveTrainSubsystems driveSub = new DriveTrainSubsystems();
  ArmAngleSubsystem armAngleSub = new ArmAngleSubsystem();
  ArmExtendSubsystem armExtendSub = new ArmExtendSubsystem();

  TestMotorSub testSub = new TestMotorSub();
  WristRotSub wristRotSub = new WristRotSub();

  DriveAuto driveAuto = new DriveAuto(driveSub);

  WristSub wristSubsystem = new WristSub();

  PathPlannerTrajectory path;

  // Trigger vacTrigger = new Trigger(() -> getRightTrigger(driver));
  // Trigger armextTrigger = new Trigger(() -> getRightTrigger(operator));

  Trigger armIn = new Trigger(() -> getLeftTrigger(operator));
  Trigger armOut = new Trigger(() -> getRightTrigger(operator));

  VacuumSubsystem vacSub = new VacuumSubsystem();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // camera = CameraServer.startAutomaticCapture();
    // MjpegServer server = new MjpegServer("Camera", 1181);
    // server.setFPS(30);
    // server.setCompression(0);
    // server.setSource(camera);

    HttpCamera limelightFeed = new HttpCamera("limelight-cavbot", "http://10.74.92.101:5800", HttpCameraKind.kMJPGStreamer);
    CameraServer.startAutomaticCapture(limelightFeed);

    driveSub.setDefaultCommand(
        new FieldDriveCommand(
            () -> modifyAxis(driver.getLeftY() *
                DriveTrainSubsystems.maxVelocityPerSecond),
            () -> modifyAxis(driver.getLeftX() *
                DriveTrainSubsystems.maxVelocityPerSecond),
            () -> modifyAxis(driver.getRightX() *
                DriveTrainSubsystems.maxAnglarVelocityPerSecond),
            driveSub));

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // JoystickButton reset = new JoystickButton(driver, 4);
    JoystickButton changeDrive = new JoystickButton(driver, 3);
    JoystickButton driveForward = new JoystickButton(driver, 1);
    JoystickButton resetGyro = new JoystickButton(driver, 4);

    driveForward.whileTrue(new ABCommand(driveSub));

    JoystickButton setVacuumCone = new JoystickButton(operator, 2);
    JoystickButton setVacuumCube = new JoystickButton(operator, 1);

    JoystickButton wristCCW = new JoystickButton(operator, 5);
    JoystickButton wristCW = new JoystickButton(operator, 6);

    POVButton armUp = new POVButton(operator, 0);

    armUp.onTrue(new ArmCommand(armAngleSub));

    // armUp.onTrue(new SequentialCommandGroup(
    //   new ArmCommand(armAngleSub),
    //   new ExtendCommand(armExtendSub)
    // ));

    armAngleSub.setDefaultCommand(new SequentialCommandGroup(
        // new HomeArmCommand(armAngleSub),
        new ArmAngleCommand(armAngleSub, operator::getLeftY)));
    // operator::getRightY));

    resetGyro.whileTrue(new InstantCommand(driveSub::zeroGyroscope));

    setVacuumCone.toggleOnTrue(new VacuumCommand(vacSub, 1));
    setVacuumCube.toggleOnFalse(new VacuumCommand(vacSub, -1));


    armOut.whileTrue(new ArmOutCommand(armExtendSub));
    armIn.whileTrue(new ArmInCommand(armExtendSub));

    wristSubsystem.setDefaultCommand(new WristCommand(wristSubsystem, operator::getRightY));

    changeDrive.toggleOnTrue(
        new FieldDriveCommand(
            () -> modifyAxis(driver.getRawAxis(1)),
            () -> modifyAxis(driver.getRawAxis(0)),
            () -> modifyAxis(driver.getRawAxis(4)),
            driveSub));

    wristCW.whileTrue(new StartEndCommand(
        () -> wristRotSub.setWrist(.3),
        () -> wristRotSub.setWrist(0),
        wristRotSub));

    wristCCW.whileTrue(new StartEndCommand(
        () -> wristRotSub.setWrist(-.3),
        () -> wristRotSub.setWrist(0),
        wristRotSub));

    // vacTrigger.whileTrue(new ArmExtendCommand(armExtendSub, () ->
    // getShouldNegate(driver)));

    // driveForward.whileTrue(new InstantCommand( () -> driveSub.drive(new
    // ChassisSpeeds(0, 10, 0))));
  }

  // public void resetOdo() {
  // driveSub.resetOdometry(path.getInitialPose());
  // }

  // public void updateOdometry() {
  // driveSub.updateOdo();
  // }

  // public Command getExtendCommand() {
  // return new AutoArmCommand(armExtendSub).withTimeout(7);
  // };

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

        new RunCommand(() -> driveSub.drive(new ChassisSpeeds(.75, 0, 0)), driveSub).withTimeout(3.256976420890),

        new InstantCommand(() -> driveSub.drive(new ChassisSpeeds(0, 0, 0)))

    // new StartEndCommand(() -> driveSub.drive(new ChassisSpeeds(1.5, 0, 0)),
    // () -> driveSub.drive(new ChassisSpeeds(0, 0, 0)),
    // driveSub).withTimeout(4),

    // new ABCommand(driveSub)
    
    );
  }

  // public ArmAngleSubsystem getArmAngleSub() {
  //   return armAngleSub;
  // }


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