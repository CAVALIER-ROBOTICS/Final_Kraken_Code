package frc.robot;

import java.util.ArrayList;
import java.util.function.Consumer;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveTrainSubsystems;

/** Add your docs here. */
public class DriveAuto  {

    DriveTrainSubsystems driveSub;
    PathPlannerTrajectory simplePath;
    PathPlannerTrajectory complexPath;
    PIDController thetaController;

    PathPlannerTrajectory path = PathPlanner.loadPath("BloobadoopV9", Constants.CONSTRAINTS);

    public DriveAuto(DriveTrainSubsystems d) {
        driveSub = d;
        thetaController = new PIDController(Constants.AutoConstants.thetaP, Constants.AutoConstants.thetaI, Constants.AutoConstants.thetaD);//.4
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }   


    public Command loadPls() {

        if(path==null) {
            SmartDashboard.putBoolean("PATH WAS NULL", true);
        }
        else {
            SmartDashboard.putBoolean("PATH WAS NULL", false);
        }
        Consumer<SwerveModuleState[]> states = a -> driveSub.setModules(driveSub.invert(a));
        return new PPSwerveControllerCommand(path, 
        driveSub::getPose, 
        Constants.m_kinematics, 
        new PIDController(Constants.AutoConstants.PIDXP, Constants.AutoConstants.PIDXI, Constants.AutoConstants.PIDXD),
        new PIDController(Constants.AutoConstants.PIDYP, Constants.AutoConstants.PIDYI, Constants.AutoConstants.PIDYD),
        thetaController,
        states,
        driveSub);
        
    }
}