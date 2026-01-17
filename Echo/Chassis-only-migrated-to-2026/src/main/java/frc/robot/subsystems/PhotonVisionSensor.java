// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.ListIterator;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** This class represents a sensor for detecting april elements of the game field using PhotonVision. */
public final class PhotonVisionSensor extends SubsystemBase {

  // The name of the network table here MUST match the name specified for the camera in the UI
  static PhotonCamera m_cameraFront = new PhotonCamera("photoncamera_front");
  static PhotonCamera m_cameraBack = new PhotonCamera("photoncamera_back");

  /// @todo Fill this in with the correct measurements later
  // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
  static Transform3d robotToCamFront = new Transform3d(new Translation3d(0.15, -.35, 0.2), 
      new Rotation3d(0,0,0));
  static Transform3d robotToCamBack = new Transform3d(new Translation3d(-0.15, .35, 0.2), 
      new Rotation3d(0,0,Math.PI));

  static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);    
  static PhotonPoseEstimator m_EstimatorFront = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamFront);
  static PhotonPoseEstimator m_EstimatorBack = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamBack);

  private EstimatedRobotPose m_latestEstimatedPose = new EstimatedRobotPose(new Pose3d(0,0,0,new Rotation3d(0,0,0)), 0,null, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
  private ShuffleboardTab m_matchTab = Shuffleboard.getTab("Match");
  private boolean m_poseEstimateAcquired = false;

  public PhotonVisionSensor() {
    // constructor
    setupDashboard();
  }

  private void setupDashboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("Photon");
    tab.addDouble("Pose X", () -> getPoseX());
    tab.addDouble("Pose Y", () -> getPoseY());
    tab.addString("Rotation", () -> getPoseRot());
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

  private Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose, 
      PhotonPoseEstimator photonPoseEstimator, PhotonCamera camera) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    List<PhotonPipelineResult> pipelineResults = camera.getAllUnreadResults();
    Optional<EstimatedRobotPose> result = Optional.empty();

    if (!pipelineResults.isEmpty()) {
      // System.out.println("MJS: non empty results in getEstimatedGlobalPose");
      int MAX_NUM_ITERATIONS = 15;

      // Set the iterator to the back of the list of results and grab the last result to begin
      ListIterator<PhotonPipelineResult> iter = pipelineResults.listIterator(pipelineResults.size());
      PhotonPipelineResult latestPipelineResult = iter.previous();

      // While:
      // 1. The current pipeline result doesn't have targets,
      // 2. There is another result to iterate to, and
      // 3. We haven't exceeded our max number of iterations, 
      // keep looking for a result with a target
      while (!latestPipelineResult.hasTargets() && 
             iter.hasPrevious() && 
             (pipelineResults.size() - iter.previousIndex() + 1) < MAX_NUM_ITERATIONS) {
        latestPipelineResult = iter.previous();
      }

      if (latestPipelineResult.hasTargets()) {
        // System.out.println("MJS: have targets in getEstimatedGlobalPose");
        result = photonPoseEstimator.update(latestPipelineResult);
        m_poseEstimateAcquired = true;
      }
    }
    return result;
  }

  public Optional<EstimatedRobotPose> getEstimatedPoseFront(Pose2d prevEstimatedRobotPose) {
    return getEstimatedGlobalPose(prevEstimatedRobotPose, m_EstimatorFront, m_cameraFront);
  }

  public Optional<EstimatedRobotPose> getEstimatedPoseBack(Pose2d prevEstimatedRobotPose) {
    return getEstimatedGlobalPose(prevEstimatedRobotPose, m_EstimatorBack, m_cameraBack);
  }

  // this is only for troubleshooting
  EstimatedRobotPose debugGetLatestEstimatedPose (Pose2d prevEstimatedRobotPose) {
    Optional<EstimatedRobotPose> poseOption = getEstimatedGlobalPose(prevEstimatedRobotPose, m_EstimatorFront, m_cameraFront);
    if (poseOption.isPresent()) {
      m_latestEstimatedPose = poseOption.get();
    } else {
      poseOption = getEstimatedGlobalPose(prevEstimatedRobotPose, m_EstimatorBack, m_cameraBack);
      if (poseOption.isPresent()) {
        m_latestEstimatedPose = poseOption.get();
      }
    }
    return m_latestEstimatedPose;
  }

  public double getPoseX(){return m_latestEstimatedPose.estimatedPose.toPose2d().getX();}
  public double getPoseY(){return m_latestEstimatedPose.estimatedPose.toPose2d().getY();}
  public String getPoseRot () {return m_latestEstimatedPose.estimatedPose.toPose2d().getRotation().toString();}
  public boolean getPoseEstimateAcquired () {return m_poseEstimateAcquired;}

}
