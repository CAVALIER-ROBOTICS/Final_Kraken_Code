// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.FileReader;
import java.io.IOException;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.Optional;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class PhotonVision {
    static PhotonCamera c = new PhotonCamera("OV5647");

    public static PhotonPipelineResult getResults() {
        return c.getLatestResult();
    }

    public static List<PhotonTrackedTarget> getAprilTags() {
        return getResults().getTargets();
    }

    public static PhotonTrackedTarget getBest(PhotonPipelineResult result) {
        return result.getBestTarget();
    }

    public static Pose3d getPose3dRelativeToAprilTag(PhotonPipelineResult results, AprilTagFieldLayout layout) throws NoSuchElementException { 
        PhotonTrackedTarget target = getBest(results);

        if(target != null) {
            Optional<Pose3d> pose = layout.getTagPose(target.getFiducialId());
            Pose3d poseBetter = pose.get();

        return PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), poseBetter, Constants.CAMERA_TO_ROBOT);
        }
        else {
            return null;
        }
     }
}
