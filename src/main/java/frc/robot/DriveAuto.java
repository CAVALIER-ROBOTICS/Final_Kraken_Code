// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Consumer;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrainSubsystems;
import frc.robot.Constants.*;

/** Add your docs here. */
public class DriveAuto {
    DriveTrainSubsystems driveSub;
    PIDController thetaController;
    PIDController xController = new PIDController(Constants.AutoConstants.PIDXP, Constants.AutoConstants.PIDXI,
            Constants.AutoConstants.PIDXD);
    PIDController yController = new PIDController(Constants.AutoConstants.PIDYP, Constants.AutoConstants.PIDYI,
            Constants.AutoConstants.PIDYP);

    public DriveAuto(DriveTrainSubsystems d) {
        driveSub = d;
        thetaController = new PIDController(Constants.AutoConstants.thetaP, Constants.AutoConstants.thetaI,
                Constants.AutoConstants.thetaI);// .4
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public PathPlannerTrajectory loadPath(String name) {
        return PathPlanner.loadPath(
                name,
                Constants.AutoConstants.maxAccelerationMetersPerSecond,
                Constants.AutoConstants.maxAccelerationMetersPerSecondSquared,
                false);
    }

    public Command getPath(String name) {
        Consumer<SwerveModuleState[]> consumer = a -> driveSub.getModuleStates();
        PPSwerveControllerCommand command = new PPSwerveControllerCommand(
                loadPath(name),
                driveSub::getPose,
                Constants.m_kinematics,
                xController,
                yController,
                thetaController,
                consumer,
                driveSub);

        // Run path following command, then stop at the end.
        return command.andThen(() -> driveSub.drive(new ChassisSpeeds(.01, 0, 0)));
    }
}
