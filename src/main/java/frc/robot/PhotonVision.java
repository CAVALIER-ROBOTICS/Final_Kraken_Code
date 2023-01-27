// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

/** Add your docs here. */
public class PhotonVision {
    static PhotonCamera c = new PhotonCamera("OV5647"); 

    public static PhotonPipelineResult getResults() {
        return c.getLatestResult();
    }

    public static List<PhotonTrackedTarget> getAprilTags() {
        return getResults().getTargets();
    }

    public static PhotonTrackedTarget getBest() {
        return getResults().getBestTarget();
    }

    public static Pose3d getPose3dRelativeToAprilTag(PhotonPipelineResult results) {
        if(results.hasTargets()) {
            AprilTagFieldLayout layout;

            try {
                layout = (AprilTagFieldLayout.loadFromResource(".wpilib\\FieldAprilTag2023.json"));

            } catch (IOException io) {
                return null;
            }

            PhotonTrackedTarget target = getBest();


            Optional<Pose3d> pose = layout.getTagPose(target.getFiducialId());
            Pose3d poseBetter = pose.get();

            return PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), poseBetter,
                Constants.CAMERA_TO_ROBOT);
    }
    else {
        return null;
    }
}
}