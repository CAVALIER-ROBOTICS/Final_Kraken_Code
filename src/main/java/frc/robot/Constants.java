// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

// 65535ms is the periodic frame status stuff

public final class Constants {

        public static final String[] fourBallAuto = {
                        "FourBallPath1",
                        "FourBallPath2"
        };

        public static final double WHEELRADMM = (3.9 * 2.54) / 2;
        public static final double NOMINAL_VOLTAGE = 12.3;
        public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)),
                        new Pose3d(0.0, .762, 0.0, new Rotation3d()));

        public static final String simpleAuto = "SimpleAuto";

        public static final double WRIST_MAX = 360; // TODO get actual values
        public static final double ARM_MAX = 360;

        public static final int wristID = 44;

        public static final int vacuum99ID = 99;
        public static final int vacuum88ID = 88;
        public static final int vacuum77ID = 77;

        public static final int armExtendID = 33;

        public static final int armAngle11ID = 11;
        public static final int armAngle22ID = 22;

        public static final int wristRotID = 55;

        public final static double L = .737;
        public final static double W = .737;

        public static final PathConstraints CONSTRAINTS = new PathConstraints(4, 3);

        public final static SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
                        // Front left
                        new Translation2d(-L / 2, -W / 2), // .4041 .5751
                        // Front right
                        new Translation2d(-L / 2, W / 2),
                        // Back right
                        new Translation2d(L / 2, W / 2),
                        // Back left
                        new Translation2d(L / 2, -W / 2));

        public static final int encoderCPR = 2048;
        // inches
        public static final double wheelDiamter = .1016;
        public static final double distancePerPulse =
                        // Assumes the encoders are directly mounted on the wheel shafts
                        (wheelDiamter * Math.PI) / (double) encoderCPR;

        public static double acceptedVolts = 65;

        public static double FIELDWIDTH = 26.0 + (7 / 12);
        public static double FIELDLENGTH = 57.0 + (1 / 12);

        // public static final double TranslationController = 1;//.7
        // public static final double StrafeController = 1;//.7
        // public static final double ThetaController = .8;//1

        // //per second radians Math.PI;
        // public static final double maxAngularSpeed = 3;
        // // per second per second in radians
        // public static final double maxAngularAcceleration = 2;

        // public static final TrapezoidProfile.Constraints thetaControllerConstraints =
        // new TrapezoidProfile.Constraints(maxAngularSpeed, maxAngularAcceleration);

        public static final class AutoConstants {
                public static final double maxSpeedMetersPerSecond = 4;
                public static final double maxAccelerationMetersPerSecondSquared = 3;

                public static final double maxAngularSpeedRadiansPerSecond = Math.PI;
                public static final double maxAngularSpeedRadiansPerSecondSquared = Math.PI;

                public static final double PIDXP = 0.007; // 0.0000000008
                public static final double PIDYP = 0.007;

                public static final double PIDXI = .0001;
                public static final double PIDXD = .0001;

                public static final double PIDYI = .0001;
                public static final double PIDYD = .0001;

                public static final double thetaP = .005; // .005
                public static final double thetaI = 0.0001;
                public static final double thetaD = 0;

                // Constraint for the motion profiled robot angle controller
                public static final TrapezoidProfile.Constraints thetaControllerConstraints = new TrapezoidProfile.Constraints(
                                maxAngularSpeedRadiansPerSecond, maxAngularSpeedRadiansPerSecondSquared);
                public static final double maxAccelerationMetersPerSecond = 0;
        }

}
