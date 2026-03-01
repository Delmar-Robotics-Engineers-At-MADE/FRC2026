// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretContainer extends SubsystemBase {
  // Create Turret
  private final TurretSubsystem m_turret = new TurretSubsystem(14, 30, 
      DriveConstants.kFrontLeftChassisAngularOffset);

  // Odometry class for tracking robot pose
  SwerveDrivePoseEstimator m_odometry = null;  // filled in by constructor

  // fusion time-of-flight sensor for homing
  FusionRangeSensor m_fusionRange = null;

  boolean m_homed = false;
  private ShuffleboardTab m_tab = Shuffleboard.getTab("Turret");

  // Constructor
  public TurretContainer(SwerveDrivePoseEstimator robot_odometry, FusionRangeSensor fusionRange) {
    m_odometry = robot_odometry;
    m_fusionRange = fusionRange; // used for homing turret
    setupDashboard();
  }

  private void setupDashboard() {
    m_tab.addBoolean("Homed", () -> getHomed()).withPosition(6, 0);
  }

  private boolean getHomed() {
    return m_homed;
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getRobotPose() {
    return m_odometry.getEstimatedPosition();
  }

  /**
   * Method to point and shoot the turret
   *
   * @param xSpeed        x component of direction to point turret
   * @param ySpeed        y component of direction to point turret
   * @param shooterSpeed  angular speed for shooter
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void rotateAndShoot(double xComponent, double yComponent, double shooterSpeed, boolean fieldRelative) {
    // Convert the commanded speeds/components into the correct units for the shooter
    double xSpeedDelivered = xComponent * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = yComponent * DriveConstants.kMaxSpeedMetersPerSecond;

    double shooterDelivered = shooterSpeed * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, 0,
                m_odometry.getEstimatedPosition().getRotation())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, 0));
    // SwerveDriveKinematics.desaturateWheelSpeeds(
    //     swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_turret.setDesiredState(swerveModuleStates[0], shooterDelivered);
  }

  /**
   * Sets the  ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void IS_THIS_EVEN_USED_setModuleStates(SwerveModuleState[] desiredStates, double shooterSpeed) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_turret.setDesiredState(desiredStates[0], shooterSpeed);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_turret.resetEncoders();
  }

}
