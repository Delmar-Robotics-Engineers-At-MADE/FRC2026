// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.AutoConstants;
// import edu.wpi.first.wpilibj.ADIS16470_IMU;
// import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.MySwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.kauailabs.navx.frc.AHRS;

public class DriveSubsystem extends SubsystemBase {

  // Constants
  static final double SlideToTheHornDistance = 0.12; // meters to slide left or right
  public static final double DriveSpeedDivider = 4;


  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS();

  // photon vision subsystem
  PhotonVisionSensor m_photon = null;

  // Odometry class for tracking robot pose
  SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0)),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()},
          Pose2d.kZero,
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
      );

  // ********* stuff for following paths in teleop ***********

  // Alliance m_allianceColor = DriverStation.getAlliance().get();

  private  Trajectory m_trajectoryForTeleop;
  private int m_aprilTargetForTeleop = 0;
  public enum HornSelection {
    L, R, Between
  }

  TrajectoryConfig m_trajectoryConfigForTeleop = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond/2,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      // Add kinematics to ensure max speed is actually obeyed
      .setKinematics(DriveConstants.kDriveKinematics);

  ProfiledPIDController m_thetaControllerForTeleop = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);

  // ************************************************************


  ShuffleboardTab m_driveBaseTab;

  private void setupDashboard() {
    m_driveBaseTab = Shuffleboard.getTab("Drivebase");
    m_driveBaseTab.add("Gyro", m_gyro);
    m_driveBaseTab.addDouble("Pose X", () -> getPoseX());
    m_driveBaseTab.addDouble("Pose Y", () -> getPoseY());
    m_driveBaseTab.addString("Rotation", () -> getPoseRot());
    m_driveBaseTab.addInteger("April Tag Targ", () -> getAprilTagForTeleop());
  }

  double getPoseX () {return m_odometry.getEstimatedPosition().getX();}
  double getPoseY () {return m_odometry.getEstimatedPosition().getY();}
  String getPoseRot () {return m_odometry.getEstimatedPosition().getRotation().toString();}

  Pose2d getEstimatedPosition() {return m_odometry.getEstimatedPosition();}

  /** Creates a new DriveSubsystem. */ // constructor
  public DriveSubsystem(PhotonVisionSensor photon) {
    m_photon = photon;

    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    // RobotConfig config = null;
    // try{
    //   config = RobotConfig.fromGUISettings();
    // } catch (Exception e) {
    //   // Handle exception as needed
    //   e.printStackTrace();
    // }

    // Configure AutoBuilder last
    // AutoBuilder.configure(
    //         this::getPose, // Robot pose supplier
    //         this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
    //         this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //         (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
    //         new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
    //                 new PIDConstants(1.0, 0.0, 0.0), // Translation PID constants
    //                 new PIDConstants(0.5, 0.0, 0.0) // Rotation PID constants
    //         ),
    //         config, // The robot configuration
    //         () -> {
    //           // Boolean supplier that controls when the path will be mirrored for the red alliance
    //           // This will flip the path being followed to the red side of the field.
    //           // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //           var alliance = DriverStation.getAlliance();
    //           if (alliance.isPresent()) {
    //             return alliance.get() == DriverStation.Alliance.Red;
    //           }
    //           return false;
    //         },
    //         this // Reference to this subsystem to set requirements
    // );

    setupDashboard();

    m_thetaControllerForTeleop.enableContinuousInput(-Math.PI, Math.PI);

  }

  SwerveModulePosition[] getCurrentPositions() {
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
    };
  }

  public void debugResetOdometryToVision (PhotonVisionSensor vision) {
    m_odometry.update(m_gyro.getRotation2d(), getCurrentPositions());
    EstimatedRobotPose pose = vision.debugGetLatestEstimatedPose(getPose());
    while (pose.timestampSeconds == 0 || Timer.getTimestamp() - pose.timestampSeconds > 0.5) {
      // keep trying until we get a fresh pose estimate
      pose = vision.debugGetLatestEstimatedPose(getPose());
    }
    resetOdometry(pose.estimatedPose.toPose2d()); // was resetPose
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0)),
        getCurrentPositions());

    // add vision data
    Optional<EstimatedRobotPose> visionOptional = m_photon.getEstimatedPoseFront(
        m_odometry.getEstimatedPosition());
    if (visionOptional.isPresent()) {
      EstimatedRobotPose visionPose = visionOptional.get(); 
      m_odometry.addVisionMeasurement(visionPose.estimatedPose.toPose2d(), visionPose.timestampSeconds);
    }
    visionOptional = m_photon.getEstimatedPoseBack(
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

  // simple version, without gyro reference
  // important to call the version with gyro, since that calculates the offset from gyro to pose
  // public void resetPose (Pose2d pose) {
  //   m_odometry.resetPose(pose);
  // }

  private SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_rearLeft.getState(),
            m_rearRight.getState()
    };
  }

  private ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  private void driveRobotRelative(ChassisSpeeds speeds) {
    drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);
  }

  /**
   * Resets the odometry to the specified pose, except use gyro.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
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
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond / DriveSpeedDivider;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond / DriveSpeedDivider;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed / DriveSpeedDivider;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(m_gyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0) ))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
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
    return Rotation2d.fromDegrees(m_gyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0) ).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  // public void setTrajectoryToProcessor(PhotonVisionSensor photon) {
  //   // resetOdometryToVision(photon);
  //   // m_trajectoryForTeleop = TrajectoryGenerator.generateTrajectory(
  //   //     new Pose2d(0, 0, new Rotation2d(0)),
  //   //     List.of(),
  //   //     new Pose2d(2, -2, new Rotation2d(0)),
  //   //     m_trajectoryConfigForTeleop);

  //   Pose2d currentPose = getPose();
  //   // first rotate to final orientation, then beeline to target
  //   m_trajectoryForTeleop = TrajectoryGenerator.generateTrajectory(
  //       currentPose,
  //       List.of(),
  //       new Pose2d(11.5, 7.0, new Rotation2d(Math.PI/2)),
  //       m_trajectoryConfigForTeleop);
        
  // }

  static final double FieldLength = 17.55;  // meters
  static final double FieldWidth = 8.05;  // meters
  public void setTrajectoryToAprilTarget(int id, HornSelection hornSelect, 
      PhotonVisionSensor photon) {
    //resetOdometryToVision(photon);
    m_aprilTargetForTeleop = id;
    double targetX = 0.0; double targetY = 0.0; double rot = 0.0;
    switch (id) {
      // case 1: targetX = 16.47987442; targetY = 0.962976864; rot = Math.toRadians(-54); break;
      case 1: targetX = 15.68; targetY = 0; rot = Math.toRadians(120); break; // for summer
      case 2: targetX = 16.47987442; targetY = 7.097023136; rot = Math.toRadians(54); break;
      case 3: targetX = 11.56; targetY = 7.6855; rot = Math.toRadians(90); break;
      case 4: targetX = 9.6545; targetY = 6.14; rot = Math.toRadians(-180); break;
      case 5: targetX = 9.6545; targetY = 1.91; rot = Math.toRadians(-180); break;
      // case 6: targetX = 13.65725; targetY = 2.98; rot = Math.toRadians(120); break;
      case 6: targetX = 13.77; targetY = 2.69; rot = Math.toRadians(-60); break; // for summer
      case 7: targetX = 14.26; targetY = 4.03; rot = Math.toRadians(180); break;
      case 8: targetX = 13.65725; targetY = 5.074326514; rot = Math.toRadians(-120); break;
      case 9: targetX = 12.45275; targetY = 5.074326514; rot = Math.toRadians(-60); break;
      case 10: targetX = 11.8555; targetY = 4.03; rot = Math.toRadians(0); break;
      case 11: targetX = 12.45; targetY = 2.99; rot = Math.toRadians(60); break;
    }
    if (hornSelect == HornSelection.R) {
      targetX += Math.sin(rot) * SlideToTheHornDistance;
      targetY -= Math.cos(rot) * SlideToTheHornDistance;
    } else if (hornSelect == HornSelection.L) {
      targetX -= Math.sin(rot) * SlideToTheHornDistance;
      targetY += Math.cos(rot) * SlideToTheHornDistance;
    }
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      // flip to cousin on other side of field
      targetX = FieldLength - targetX;
      targetY = FieldWidth - targetY;
      rot = (rot < 0) ? rot + Math.PI : rot - Math.PI;
    }
    Pose2d currentPose = getPose();
    m_trajectoryForTeleop = TrajectoryGenerator.generateTrajectory(
        currentPose,
        List.of(),
        new Pose2d(targetX, targetY, new Rotation2d(rot)),
        m_trajectoryConfigForTeleop);
        
  }

  public void setTrajectoryToRotateDownfield(PhotonVisionSensor photon) {
    Pose2d currentPose = getPose();
    m_trajectoryForTeleop = TrajectoryGenerator.generateTrajectory(
        currentPose,
        List.of(),
        new Pose2d(currentPose.getX(), currentPose.getY(), new Rotation2d(0)),
        m_trajectoryConfigForTeleop);
        
  }
  
  // public Command setTrajectoryToProcessorCmd(PhotonVisionSensor photon) {
  //   return new InstantCommand(() -> setTrajectoryToProcessor(photon));
  // }
  public Command setTrajectoryToAprilTargetCmd(int id, HornSelection hornSelect,
      PhotonVisionSensor photon) {
    return new InstantCommand(() -> setTrajectoryToAprilTarget(id, hornSelect, photon));
  }
  public Command setTrajectoryToRotateDownfieldCmd(PhotonVisionSensor photon) {
    return new InstantCommand(() -> setTrajectoryToRotateDownfield(photon));
  }

  public Command getSwerveControllerCmdForTeleop(PhotonVisionSensor photon) {
    return new MySwerveControllerCommand(
        this::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        m_thetaControllerForTeleop,
        this::setModuleStates,
        this, photon,
        this);
  }

  public Trajectory getTrajectoryForTeleop() {
    return m_trajectoryForTeleop;
  }

  public int getAprilTagForTeleop() {
    return m_aprilTargetForTeleop;
  }

  public Command setXCommand() {
    return new InstantCommand(() -> setX());
  }
  public Command setXCommandHold() {
    return new RunCommand(() -> setX());
  }



}
