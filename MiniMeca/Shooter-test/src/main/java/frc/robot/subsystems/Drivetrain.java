// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.kauailabs.navx.frc.AHRS;

class myAHRS extends AHRS {
  @Override
  public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getAngle());
  }

  public myAHRS(SPI.Port kmxp, byte update_rate_hz) {
     super(kmxp, update_rate_hz);
  }
}

class DriveConstants {
  public static final int kEncoderCPR = 74;
  public static final double kWheelDiameterMeters = 0.10;
  public static final double kEncoderDistancePerPulse =
      // Assumes the encoders are directly mounted on the wheel shafts
      (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;
}

/** Represents a mecanum drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Talon m_frontLeftMotor = new Talon(1);
  private final Talon m_frontRightMotor = new Talon(3);
  private final Talon m_backLeftMotor = new Talon(0);
  private final Talon m_backRightMotor = new Talon(2);

  private final Encoder m_frontLeftEncoder = new Encoder(2, 3);
  private final Encoder m_frontRightEncoder = new Encoder(6, 7);
  private final Encoder m_backLeftEncoder = new Encoder(0, 1);
  private final Encoder m_backRightEncoder = new Encoder(4, 5);

  // private final Translation2d m_frontLeftLocation = new Translation2d(0.1588, 0.1651);
  // private final Translation2d m_frontRightLocation = new Translation2d(0.1588, -0.1651);
  // private final Translation2d m_backLeftLocation = new Translation2d(-0.1588, 0.1651);
  // private final Translation2d m_backRightLocation = new Translation2d(-0.1588, -0.1651);
  private final Translation2d m_frontLeftLocation = new Translation2d(-0.1588, 0.1651);
  private final Translation2d m_frontRightLocation = new Translation2d(0.1588, 0.1651);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.1588, -0.1651);
  private final Translation2d m_backRightLocation = new Translation2d(0.1588, -0.1651);

  private final PIDController m_frontLeftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_frontRightPIDController = new PIDController(1, 0, 0);
  private final PIDController m_backLeftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_backRightPIDController = new PIDController(1, 0, 0);

  private final myAHRS m_gyro = new myAHRS(SPI.Port.kMXP, (byte) 200);

  private final MecanumDriveKinematics m_kinematics =
      new MecanumDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  // private final MecanumDriveOdometry m_odometry =
  //     new MecanumDriveOdometry(m_kinematics, m_gyro.getRotation2d(), getCurrentDistances());

  // use this instead of MecanumDriveOdometry, per reference
  private final MecanumDrivePoseEstimator m_poseEstimator =
    new MecanumDrivePoseEstimator(
        m_kinematics,
        m_gyro.getRotation2d(),
        getCurrentDistances(),
        Pose2d.kZero,
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));


// Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

  /** Constructs a MecanumDrive and resets the gyro. */
  public Drivetrain() {
    m_gyro.reset();
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_backRightMotor.setInverted(true);
    m_backLeftMotor.setInverted(true);

    m_backRightEncoder.setReverseDirection(true);
    m_frontRightEncoder.setReverseDirection(true);

    m_frontLeftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_backLeftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_frontRightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_backRightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);


    ShuffleboardTab driveBaseTab = Shuffleboard.getTab("Drivebase");
    driveBaseTab.add("Gyro", m_gyro);

    // Put both encoders in a list layout
    ShuffleboardLayout encoders =
        driveBaseTab.getLayout("Encoders", BuiltInLayouts.kList).withPosition(4, 0).withSize(2, 4);
    encoders.add("Front Left Encoder", m_frontLeftEncoder);
    encoders.add("Front Right Encoder", m_frontRightEncoder);
    encoders.add("Rear Left Encoder", m_backLeftEncoder);
    encoders.add("Rear Right Encoder", m_backRightEncoder);    
  }

  /**
   * Returns the current state of the drivetrain.
   *
   * @return The current state of the drivetrain.
   */
  public MecanumDriveWheelSpeeds getCurrentState() {
    return new MecanumDriveWheelSpeeds(
        m_frontLeftEncoder.getRate(),
        m_frontRightEncoder.getRate(),
        m_backLeftEncoder.getRate(),
        m_backRightEncoder.getRate());
  }

  /**
   * Returns the current distances measured by the drivetrain.
   *
   * @return The current distances measured by the drivetrain.
   */
  public MecanumDriveWheelPositions getCurrentDistances() {
    return new MecanumDriveWheelPositions(
        m_frontLeftEncoder.getDistance(),
        m_frontRightEncoder.getDistance(),
        m_backLeftEncoder.getDistance(),
        m_backRightEncoder.getDistance());
  }

  /**
   * Set the desired speeds for each wheel.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(MecanumDriveWheelSpeeds speeds) {
    final double frontLeftFeedforward = m_feedforward.calculate(speeds.frontLeftMetersPerSecond);
    final double frontRightFeedforward = m_feedforward.calculate(speeds.frontRightMetersPerSecond);
    final double backLeftFeedforward = m_feedforward.calculate(speeds.rearLeftMetersPerSecond);
    final double backRightFeedforward = m_feedforward.calculate(speeds.rearRightMetersPerSecond);

    final double frontLeftOutput =
        m_frontLeftPIDController.calculate(
            m_frontLeftEncoder.getRate(), speeds.frontLeftMetersPerSecond);
    final double frontRightOutput =
        m_frontRightPIDController.calculate(
            m_frontRightEncoder.getRate(), speeds.frontRightMetersPerSecond);
    final double backLeftOutput =
        m_backLeftPIDController.calculate(
            m_backLeftEncoder.getRate(), speeds.rearLeftMetersPerSecond);
    final double backRightOutput =
        m_backRightPIDController.calculate(
            m_backRightEncoder.getRate(), speeds.rearRightMetersPerSecond);

    m_frontLeftMotor.setVoltage(frontLeftOutput + frontLeftFeedforward);
    m_frontRightMotor.setVoltage(frontRightOutput + frontRightFeedforward);
    m_backLeftMotor.setVoltage(backLeftOutput + backLeftFeedforward);
    m_backRightMotor.setVoltage(backRightOutput + backRightFeedforward);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    var mecanumDriveWheelSpeeds =
        m_kinematics.toWheelSpeeds(
            ChassisSpeeds.discretize(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                                        xSpeed, ySpeed, rot, 
                                                        m_poseEstimator.getEstimatedPosition().getRotation())
                                                   : new ChassisSpeeds(xSpeed, ySpeed, rot),
                                     periodSeconds));
    mecanumDriveWheelSpeeds.desaturate(kMaxSpeed);
    setSpeeds(mecanumDriveWheelSpeeds);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_poseEstimator.update(m_gyro.getRotation2d(), getCurrentDistances());
    // Optional<EstimatedRobotPose> visionOptional = PhotonVisionSensor.getEstimatedGlobalPose(m_poseEstimator.getEstimatedPosition());
    
    // if (visionOptional.isPresent()) {
    //   EstimatedRobotPose visionPose = visionOptional.get(); 

    //   System.out.println("X: " + String.format("%.6f", visionPose.estimatedPose.toPose2d().getX()) + "Y: " + String.format("%.6f", visionPose.estimatedPose.toPose2d().getY()));

    //   m_poseEstimator.addVisionMeasurement(
    //     visionPose.estimatedPose.toPose2d(),
    //     visionPose.timestampSeconds);
    // } else {
      System.out.println("No odometry update available");
    // }
  }
}
