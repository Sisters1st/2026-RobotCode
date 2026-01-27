package frc.robot.Logic;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Settings;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision {

        boolean updateHeadingWithVision = true;

        boolean useEoghanCam = true;

        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        PhotonCamera eoghanCam = new PhotonCamera("EoghanCam");

        public double timeATLastSeen;

        public double visionMaxATDist = Settings.VisionSettings.maxATDistDisabeled;

        Transform3d eoghanCamTransform = new Transform3d(
                        new Translation3d(Units.inchesToMeters(12), Units.inchesToMeters(8),
                                        Units.inchesToMeters(7)),
                        new Rotation3d(0, 0, Units.degreesToRadians(180)));

        PhotonPoseEstimator eoghanPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, eoghanCamTransform);

        Field2d photonField2d_processor = new Field2d();

        public void updatePhotonVision() {

                integrateCamera(useEoghanCam, eoghanCam, eoghanPoseEstimator, photonField2d_processor,
                                visionMaxATDist, false);

        }

        public void integrateCamera(boolean useCamera, PhotonCamera camera, PhotonPoseEstimator estimator,
                        Field2d photonField, double maxDistance, boolean updateLastTimeSeen) {
                for (var result : camera.getAllUnreadResults()) {
                        Optional<EstimatedRobotPose> photonPose = estimator.estimateLowestAmbiguityPose(result);
                        System.out.println("Is photon pose present: " + photonPose.isPresent());
                        if (photonPose.isPresent()) {
                                photonField.setRobotPose(photonPose.get().estimatedPose.toPose2d());

                                double tag0Dist = result.getBestTarget().bestCameraToTarget
                                                .getTranslation()
                                                .getNorm();
                                SmartDashboard.putNumber("Tag 0", tag0Dist);

                                double poseAmbiguitiy = result.getBestTarget().getPoseAmbiguity();
                                if (useCamera && tag0Dist < maxDistance && poseAmbiguitiy < 0.15) {
                                        if (updateHeadingWithVision) {
                                               
                                        }
                                }

                        }
                }
        }
}
