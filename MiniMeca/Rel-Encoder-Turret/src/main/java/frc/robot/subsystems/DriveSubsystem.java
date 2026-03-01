// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.kauailabs.navx.frc.AHRS;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  // private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
  //     /* DriveConstants.kFrontLeftDrivingCanId, */
  //     DriveConstants.kFrontLeftTurningCanId,
  //     DriveConstants.kFrontLeftChassisAngularOffset);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS();

  // photon vision subsystem
  PhotonVisionSensor m_photon = null;

  // Odometry class for tracking robot pose
  SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0)),
      new SwerveModulePosition[] {},
          Pose2d.kZero,
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
      );

  ShuffleboardTab m_driveBaseTab;

  private void setupDashboard() {
    m_driveBaseTab = Shuffleboard.getTab("Drivebase2");
    m_driveBaseTab.add("Gyro", m_gyro);
    m_driveBaseTab.addDouble("Pose X", () -> getPoseX());
    m_driveBaseTab.addDouble("Pose Y", () -> getPoseY());
    m_driveBaseTab.addString("Rotation", () -> getPoseRot());
  }

  double getPoseX () {return m_odometry.getEstimatedPosition().getX();}
  double getPoseY () {return m_odometry.getEstimatedPosition().getY();}
  String getPoseRot () {return m_odometry.getEstimatedPosition().getRotation().toString();}

  Pose2d getEstimatedPosition() {return m_odometry.getEstimatedPosition();}

  SwerveModulePosition[] getCurrentPositions() {
    return new SwerveModulePosition[] {};
  }

  public void debugResetOdometryToVision (PhotonVisionSensor vision) {
    System.out.println("---> Resetting odometry to vision");
    m_odometry.update(m_gyro.getRotation2d(), getCurrentPositions());
    EstimatedRobotPose pose = vision.debugGetLatestEstimatedPose(getPose());
    // while (pose.timestampSeconds == 0 || Timer.getTimestamp() - pose.timestampSeconds > 0.1) {
    //   // keep trying until we get a fresh pose estimate
    //   pose = vision.debugGetLatestEstimatedPose(getPose());
    // }
    resetOdometry(pose.estimatedPose.toPose2d()); // was resetPose
    System.out.println("---> Reset odometry to vision snapshot of " + (Timer.getTimestamp() - pose.timestampSeconds) + " secs ago");
  }

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(PhotonVisionSensor photon) {
    m_photon = photon;
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    setupDashboard();
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0)),
        new SwerveModulePosition[] {});

    // add vision data
    Optional<EstimatedRobotPose> visionOptional = m_photon.getEstimatedPoseFront(
        m_odometry.getEstimatedPosition());
    if (visionOptional.isPresent()) {
      EstimatedRobotPose visionPose = visionOptional.get(); 
      m_odometry.addVisionMeasurement(visionPose.estimatedPose.toPose2d(), visionPose.timestampSeconds);
    }

  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0)),
        new SwerveModulePosition[] {},
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void beATurret() {

    // calculate angle to red target, and then pretend joystick is pointing that way
    Pose2d pose = m_odometry.getEstimatedPosition();
    double deltaX = 11.92 - pose.getX();
    double deltaY = 4.03 - pose.getY();

    // normalize so one is 1 and the other is < 1
    double maxxy = Math.max(Math.abs(deltaX), Math.abs(deltaY));
    double xSpeed = deltaX / maxxy;
    double ySpeed = deltaY / maxxy;

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = 0;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, pose.getRotation()));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    // m_frontLeft.setDesiredState(swerveModuleStates[0]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    // m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    // m_frontLeft.setDesiredState(desiredStates[0]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    // m_frontLeft.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return 
      Rotation2d.fromDegrees(m_gyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0)).getDegrees();
  }


  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
