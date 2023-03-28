// Author: UMN Robotics Ri3d
// Last Updated : January 2023

package frc.robot.commands.DriveCommands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Drive.DriveTrainSubsystems;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

// This command self=balances on the charging station using gyroscope pitch as feedback
public class ABV2Command extends CommandBase {

  private DriveTrainSubsystems m_DriveSubsystem;

  private double error;
  private double currentAngle;
  private double drivePower;

  /**
   * Command to use Gyro data to resist the tip angle from the beam - to stabalize
   * and balanace
   */
  public ABV2Command(DriveTrainSubsystems dSub) {
    m_DriveSubsystem = dSub;
    addRequirements(m_DriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Uncomment the line below this to simulate the gyroscope axis with a
    // controller joystick
    // Double currentAngle = -1 *
    // Robot.controller.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS) * 45;
    this.currentAngle = m_DriveSubsystem.getPitch();

    error = 0 - currentAngle;
    drivePower = -Math.min(.05 * error, 1);

    // Our robot needed an extra push to drive up in reverse, probably due to weight
    // imbalances
    if (drivePower < 0) {
      drivePower *= 2;
    }

    // Limit the max power
    if (Math.abs(drivePower) > .8) {
      drivePower = Math.copySign(0.8, drivePower);
    }

    m_DriveSubsystem.fieldOrientedDriveNumber(0.0, -drivePower, 0.0);

    SmartDashboard.putNumber("Current Angle", currentAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(error) < 1; // End the command when we are within the specified threshold of being 'flat'
                                // (gyroscope pitch of 0 degrees)
    // return false;
  }
}