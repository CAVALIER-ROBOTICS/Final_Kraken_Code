package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Interpolation.InterpolatingTable;

/** Add your docs here. */
public class Limelight2 {
    public static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-cavbots");

    public static NetworkTableEntry tx = table.getEntry("tx");
    public static NetworkTableEntry ty = table.getEntry("ty");
    public static NetworkTableEntry ta = table.getEntry("ta");
    public static NetworkTableEntry tRobotPose = table.getEntry("botpose");

    public static double x;
    public static double y;
    public static double area;
    public static double[] robotPos;

    public static double[] defaultReturn = {
        0.0,
        0.0
    };
    
    public Limelight2() {
        // table = NetworkTableInstance.getDefault().getTable("limelight-cavbots");
        // tx = table.getEntry("tx");
        // ty = table.getEntry("ty");
        // ta = table.getEntry("ta");

        //read values periodically
        // x = tx.getDouble(0.0);
        // y = ty.getDouble(0.0);
        // area = ta.getDouble(0.0);
    }

    public static void updateValues() {
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tRobotPose = table.getEntry("botpose");

        //read values periodically
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);
        robotPos = tRobotPose.getDoubleArray(defaultReturn);

        SmartDashboard.putNumber("lime Dist.", getDistance());
        // SmartDashboard.putNumber("X v",x);
        // SmartDashboard.putNumber("Y v", y);
        // SmartDashboard.putNumber("Area values", area);
    }

    public static Pose3d estimatePosition(double[] arrayToConvert) {

        if(arrayToConvert.length > 1) {
            double xPos, yPos, zPos, xRot, yRot, zRot;
            xPos = arrayToConvert[0];
            yPos = arrayToConvert[1];
            zPos = arrayToConvert[2];
            xRot = Units.degreesToRadians(arrayToConvert[3]);
            yRot = Units.degreesToRadians(arrayToConvert[4]);
            zRot = Units.degreesToRadians(arrayToConvert[5]);

            return new Pose3d(xPos, yPos, zPos, new Rotation3d(xRot, yRot, zRot));
        }
        else {
            return null;
        }
        //return null;
    } 

    public static double getX()
    {
        return x;
    }

    public static double getY()
    {
        return y;
    }

    public static double getArea()
    {
        return area;
    }

    public static double getDistance() {
        return ((2.74-.768) / (Math.tan(Math.toRadians(48+getY()))));
    }

    public static double getRPM()
    {
        return InterpolatingTable.getShotParameter(getDistance()).getRPM();
    }

    public static double getAngle()
    {
        return InterpolatingTable.getShotParameter(getDistance()).getHoodAngle();
    }
}