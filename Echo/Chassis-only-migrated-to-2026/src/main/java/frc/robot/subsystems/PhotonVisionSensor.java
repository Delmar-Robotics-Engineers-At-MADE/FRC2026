// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** This class represents a sensor for detecting april elements of the game field using PhotonVision. */
public final class PhotonVisionSensor extends SubsystemBase {

  private SwerveDrivePoseEstimator m_odometry = null;

  // The name of the network table here MUST match the name specified for the camera in the UI
  static PhotonCamera m_cameraFront = new PhotonCamera("Arducam_OV9281_8077_Front");
  static PhotonCamera m_cameraBack = new PhotonCamera("Arducam_OV9281_8077_Rear");

  // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
  static Transform3d robotToCamFront = new Transform3d(new Translation3d(0.3048, 0.1778, 0.3048), 
      new Rotation3d(0,0,0));
  static Transform3d robotToCamBack = new Transform3d(new Translation3d(-0.3302, 0.1016, 0.3048), 
      new Rotation3d(0,0,Math.PI));

  static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);  
  static PhotonPoseEstimator m_EstimatorFront = new PhotonPoseEstimator(aprilTagFieldLayout, robotToCamFront);
  static PhotonPoseEstimator m_EstimatorBack = new PhotonPoseEstimator(aprilTagFieldLayout, robotToCamBack);

  private EstimatedRobotPose m_latestEstimatedPose = new EstimatedRobotPose(new Pose3d(0,0,0,new Rotation3d(0,0,0)), 0,null, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
  private ShuffleboardTab m_matchTab = Shuffleboard.getTab("Match");
  private boolean m_poseEstimateAcquired = false;
  private boolean m_debugTakeSnapshot = false;

  public PhotonVisionSensor(SwerveDrivePoseEstimator odometry) {

    // constructor
    setupDashboard();

    m_odometry = odometry;
  }

  private void setupDashboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("Photon");
    tab.addDouble("Snapshot X", () -> getPoseX());
    tab.addDouble("Snapshot Y", () -> getPoseY());
    tab.addString("Snapshot Rotation", () -> getPoseRot());
    m_matchTab.addBoolean("Vision Fix", () -> getPoseEstimateAcquired())
        .withPosition(6, 0);
  }  

  // public void getBestTargetLocation() {
  //   // Gets all unread results and pulls out an iterator to the last received result
  //   List<PhotonPipelineResult> results = cameraFront.getAllUnreadResults();

  //   PhotonPipelineResult latestResult;
  //   if (!results.isEmpty()) {
  //     ListIterator<PhotonPipelineResult> iter = results.listIterator();
      
  //     while (iter.hasNext()) {
  //       PhotonPipelineResult temp = iter.next();
  //       if (temp.hasTargets()) {
  //         latestResult = temp;
  //         break;
  //       }
  //     }
  //   }

  //   /// @todo Return the position of the best target in "latestResult"

  // }

  /**
   * Get a "noisy" fake global pose reading.
   *
   * @param estimatedRobotPose The robot pose.
   */
  // public static Pose2d getEstimatedGlobalPose(Pose2d estimatedRobotPose) {
  //   var rand =
  //       StateSpaceUtil.makeWhiteNoiseVector(VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
  //   return new Pose2d(
  //       estimatedRobotPose.getX() + rand.get(0, 0),
  //       estimatedRobotPose.getY() + rand.get(1, 0),
  //       estimatedRobotPose.getRotation().plus(new Rotation2d(rand.get(2, 0))));
  // }

  // private Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose, 
  //                                                             PhotonPoseEstimator photonPoseEstimator, 
  //                                                             PhotonCamera camera) {

  //   // Get all available unread results
  //   List<PhotonPipelineResult> pipelineResults = camera.getAllUnreadResults();
  //   Optional<EstimatedRobotPose> result = Optional.empty();

  //   // If results are present, look for targets to try and get a pose estimate
  //   if (!pipelineResults.isEmpty()) {

  //     photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);

  //     // Set an arbitrary maximum number of iterations so that if the results
  //     // buffer gets large, we don't spend lots of time looking for targets if there are none
  //     // Leaving this out causes the system to get hung up when in scenarios where april tags are not present
  //     int MAX_NUM_ITERATIONS = 15;

  //     // Set the iterator to the back of the list of results and grab the last result to begin
  //     ListIterator<PhotonPipelineResult> iter = pipelineResults.listIterator(pipelineResults.size());
  //     PhotonPipelineResult latestPipelineResult = iter.previous();

  //     // While:
  //     // 1. The current pipeline result doesn't have targets,
  //     // 2. There is another result to iterate to, and
  //     // 3. We haven't exceeded our max number of iterations, 
  //     // keep looking for a result with a target
  //     while (!latestPipelineResult.hasTargets() && 
  //            iter.hasPrevious() && 
  //            (pipelineResults.size() - iter.previousIndex() + 1) < MAX_NUM_ITERATIONS) {
  //       latestPipelineResult = iter.previous();
  //     }

  //     // If we were able to find a frame with targets, use that frame to estimate a global pose
  //     if (latestPipelineResult.hasTargets()) {
  //       // System.out.println("---> Pipeline has targets; calling mutitag pose");
  //       result = photonPoseEstimator.estimateCoprocMultiTagPose(latestPipelineResult);
  //       if (result.isPresent()) {
  //         m_poseEstimateAcquired = true;
  //         // EstimatedRobotPose visionPose = result.get(); 
  //         // System.out.println("X: " + String.format("%.6f", visionPose.estimatedPose.toPose2d().getX()) + " Y: " + String.format("%.6f", visionPose.estimatedPose.toPose2d().getY()));
  //         if (m_debugTakeSnapshot) {
  //           m_latestEstimatedPose = result.get();
  //           m_debugTakeSnapshot = false;
  //         }
  //       } // else {
  //       //   System.out.println("---> Multitag pose did NOT return a pose");
  //       // }
  //     }
  //   } 
  //   return result;
  // }

  private Matrix<N3, N1> calculateStdDevs(EstimatedRobotPose est, List<PhotonTrackedTarget> targets) {
    // Default to high uncertainty
    var stdDevs = VecBuilder.fill(4.0, 4.0, 8.0);

    int numTags = 0;
    double avgDist = 0.0;

    for (var target : targets) {
        var tagPose = m_EstimatorFront.getFieldTags().getTagPose(target.getFiducialId());
        if (tagPose.isEmpty()) continue;

        numTags++;
        avgDist += tagPose.get().toPose2d().getTranslation()
            .getDistance(est.estimatedPose.toPose2d().getTranslation());
    }

    if (numTags == 0) return stdDevs;
    avgDist /= numTags;

    if (numTags > 1) {
        // Multi-tag — high trust
        stdDevs = VecBuilder.fill(0.5, 0.5, 1.0);
    }

    // Scale up uncertainty with distance
    stdDevs = stdDevs.times(1.0 + (avgDist * avgDist / 30.0));

    // Reject single-tag estimates that are too far away
    if (numTags == 1 && avgDist > 4.0) {
        stdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }

    return stdDevs;
  }

  private void updateVisionEsimtate(PhotonPoseEstimator photonPoseEstimator, 
                                    PhotonCamera camera) {
    for (var result : camera.getAllUnreadResults()) {
      var visionEst = photonPoseEstimator.estimateCoprocMultiTagPose(result);

        if (visionEst.isEmpty()) {
          visionEst = photonPoseEstimator.estimatePnpDistanceTrigSolvePose(result);
        }

        visionEst.ifPresent(est -> {
            m_poseEstimateAcquired = true;

            var stdDevs = calculateStdDevs(est, result.getTargets());
            m_odometry.addVisionMeasurement(
                est.estimatedPose.toPose2d(),
                est.timestampSeconds,
                stdDevs
            );
        });
    }
  }

  // do this at least twice, until the pose snapshot is not changing anymore
  EstimatedRobotPose debugGetLatestEstimatedPose (Pose2d prevEstimatedRobotPose) {
    m_debugTakeSnapshot = true;
    return m_latestEstimatedPose;
  }

  public double getPoseX(){return m_latestEstimatedPose.estimatedPose.toPose2d().getX();}
  public double getPoseY(){return m_latestEstimatedPose.estimatedPose.toPose2d().getY();}
  public String getPoseRot () {return m_latestEstimatedPose.estimatedPose.toPose2d().getRotation().toString();}
  public boolean getPoseEstimateAcquired () {return m_poseEstimateAcquired;}


  @Override
  public void periodic() {
    // Update the vision estimates at a periodic rate
    updateVisionEsimtate(m_EstimatorFront, m_cameraFront);
    updateVisionEsimtate(m_EstimatorBack, m_cameraBack);
  }
}
