package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ShooterSubsystemConstants;

public final class Configs {

  private static final double nominalVoltage = 12.0;

  public static final class MAXSwerveModule {
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static {
      // Use module constants to calculate conversion factors and feed forward gain.
      double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI 
        / ModuleConstants.kDrivingMotorReduction;
      double turningFactor = 2 * Math.PI;
      double drivingVelocityFeedForward = nominalVoltage / ModuleConstants.kDriveWheelFreeSpeedRps;
      
      drivingConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(50);

      drivingConfig.encoder
        .positionConversionFactor(drivingFactor) // meters
        .velocityConversionFactor(drivingFactor / 60.0); // meters per second

      drivingConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // These are example gains you may need to them for your own robot!
        .pid(0.04, 0, 0)
        .outputRange(-1, 1)
        .feedForward.kV(drivingVelocityFeedForward);

      turningConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(20);

      turningConfig
        .absoluteEncoder
          // Invert the turning encoder, since the output shaft rotates in the opposite
          // direction of the steering motor in the MAXSwerve Module.
          .inverted(true)
          .positionConversionFactor(turningFactor) // radians
          .velocityConversionFactor(turningFactor / 60.0) // radians per second
          // This applies to REV Through Bore Encoder V2 (use REV_ThroughBoreEncoder for V1):
          .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoder);
        
      turningConfig
        .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          // These are example gains you may need to them for your own robot!
          .pid(1, 0, 0)
          .outputRange(-1, 1)
          // Enable PID wrap around for the turning motor. This will allow the PID
          // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
          // to 10 degrees will go through 0 rather than the other direction which is a
          // longer route.
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0, turningFactor);
    }
  }

  public static final class IntakeSubsystem {
    public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();
    public static final SparkMaxConfig conveyorConfig = new SparkMaxConfig();

    static {
    // Configure basic settings of the intake motor
    intakeConfig
      .inverted(false)
      .idleMode(IdleMode.kCoast)
      .openLoopRampRate(1.0)
      .smartCurrentLimit(40);

    // Configure basic settings of the conveyor motor
    conveyorConfig
      .inverted(true)
      .idleMode(IdleMode.kCoast)
      .openLoopRampRate(1.0)
      .smartCurrentLimit(40);
    }
  }

  public static final class ShooterSubsystem {
    public static final SparkMaxConfig flywheelConfig = new SparkMaxConfig();
    public static final SparkMaxConfig flywheelFollowerConfig = new SparkMaxConfig();
    public static final SparkMaxConfig feederConfig = new SparkMaxConfig();

    public static final SparkMaxConfig turretYawConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turretPitchConfig = new SparkMaxConfig();

    static {
      // Configure basic setting of the flywheel motors
      flywheelConfig
        .idleMode(IdleMode.kCoast)
        .closedLoopRampRate(1.0)
        .openLoopRampRate(1.0)
        .smartCurrentLimit(40);

      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      flywheelConfig
        .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for velocity control
          .p(0)
          .outputRange(-1, 1);

      flywheelConfig.closedLoop
        .maxMotion
          // Set MAXMotion parameters for MAXMotion Velocity control
          // CruiseVelocity is not included here as it is specifically called out in the docs to only affect position control
          .maxAcceleration(3000) // rpm/s
          .allowedProfileError(ShooterSubsystemConstants.FlywheelSetpoints.kVelocityTolerance); // rpm

      // Constants.NeoMotorConstants.kVortexKv is in rpm/V. feedforward.kV is in V/rpm sort we take
      // the reciprocol.
      flywheelConfig.closedLoop
        .feedForward
          .kV(0.0020215101); // V/rpm; from ReCalc (0.35 V*s/m converted to V/rpm)
          //.kA(0.0018619172); // V/(rpm/s); from ReCalc (0.32 V*s^2/m converted to V/(rpm/s))

      // Configure the follower flywheel motor to follow the main flywheel motor
      flywheelFollowerConfig.apply(flywheelConfig)
        .follow(Constants.ShooterSubsystemConstants.kFlywheelMotorCanId, true);

      // Configure basic setting of the feeder motor
      feederConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .openLoopRampRate(1.0)
        .smartCurrentLimit(30);

      turretYawConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .openLoopRampRate(1.0)
        .closedLoopRampRate(1.0)
        .smartCurrentLimit(30);

       /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      turretYawConfig
        .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for velocity control
          .p(0)
          .outputRange(-1, 1);

      turretYawConfig.closedLoop
        .maxMotion
          // Set MAXMotion parameters for MAXMotion Position control
          .cruiseVelocity(5000) // rpm
          .maxAcceleration(3000) // rpm/s
          .allowedProfileError(ShooterSubsystemConstants.TurretSetpoints.kYawPositionTolerance) // rotations
          // Set MAXMotion parameters for MAXMotion Velocity control
          // CruiseVelocity is not included here as it is specifically called out in the docs to only affect position control
          .maxAcceleration(3000, ClosedLoopSlot.kSlot1) // rpm/s
          .allowedProfileError(ShooterSubsystemConstants.TurretSetpoints.kYawVelocityTolerance, ClosedLoopSlot.kSlot1); // rotations

      // Motor kV is 1/motor free speed rpm. Feedforward config expects this value as a factor of V/rpm so multiply by
      // the nominal voltage
      turretYawConfig.closedLoop
        .feedForward
          .kV(nominalVoltage / Constants.NeoMotorConstants.kFreeSpeedRpm); // rpm

      turretPitchConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .openLoopRampRate(1.0)
        .closedLoopRampRate(1.0)
        .smartCurrentLimit(30);

      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      turretPitchConfig
        .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for velocity control
          .p(0)
          .outputRange(-1, 1);

      turretPitchConfig.closedLoop
        .maxMotion
          // Set MAXMotion parameters for MAXMotion Position control
          .cruiseVelocity(5000) // rpm
          .maxAcceleration(3000) // rpm/s
          .allowedProfileError(ShooterSubsystemConstants.TurretSetpoints.kYawPositionTolerance) // rotations
          // Set MAXMotion parameters for MAXMotion Velocity control
          // CruiseVelocity is not included here as it is specifically called out in the docs to only affect position control
          .maxAcceleration(3000, ClosedLoopSlot.kSlot1) // rpm/s
          .allowedProfileError(ShooterSubsystemConstants.TurretSetpoints.kPitchVelocityTolerance, ClosedLoopSlot.kSlot1); // rotations

      // Motor kV is 1/motor free speed rpm. Feedforward config expects this value as a factor of V/rpm so multiply by
      // the nominal voltage
      turretPitchConfig.closedLoop
        .feedForward
          .kV(nominalVoltage / Constants.NeoMotorConstants.kFreeSpeedRpm); // rpm
    }
  }
}
