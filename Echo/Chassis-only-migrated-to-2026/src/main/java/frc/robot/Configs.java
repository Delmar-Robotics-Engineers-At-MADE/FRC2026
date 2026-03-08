package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.*;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.LightsSubsystemConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.Neo550MotorConstants;
import frc.robot.Constants.ShooterSubsystemConstants;
import frc.robot.Constants.TurretSubsystemConstants.TurretSetpoints;
import frc.robot.Constants.TurretSubsystemConstants.TurretUnits;

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
        .smartCurrentLimit(Neo550MotorConstants.kMaxAllowedCurrent);

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
      .inverted(false)
      .idleMode(IdleMode.kCoast)
      .openLoopRampRate(1.0)
      .smartCurrentLimit(Neo550MotorConstants.kMaxAllowedCurrent);
    }
  }

  public static final class TurretSubsystem {
    public static final SparkMaxConfig turretYawConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turretPitchConfig = new SparkMaxConfig();

    static {

      turretYawConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .openLoopRampRate(1.0)
        .closedLoopRampRate(1.0)
        .smartCurrentLimit(Neo550MotorConstants.kMaxAllowedCurrent - 5, Neo550MotorConstants.kMaxAllowedCurrent)
        .secondaryCurrentLimit(Neo550MotorConstants.kMaxAllowedCurrent, 20);

      // turretYawConfig
      //   .softLimit
      //     .forwardSoftLimit(TurretSetpoints.kYawMotorMaxSetpoint)
      //     .forwardSoftLimitEnabled(true)
      //     .reverseSoftLimit(TurretSetpoints.kYawMotorMinSetpoint)
      //     .reverseSoftLimitEnabled(true);

      turretYawConfig
        .encoder
          .positionConversionFactor(TurretUnits.kYawPositionConversionFactor)
          .velocityConversionFactor(TurretUnits.kYawVelocityConversionFactor);

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
          .cruiseVelocity(1500 * TurretUnits.kYawVelocityConversionFactor) // degrees per sec
          .maxAcceleration(1000 * TurretUnits.kYawVelocityConversionFactor) // degrees per sec/s
          .allowedProfileError(TurretSetpoints.kYawPositionTolerance) // degrees
          // Set MAXMotion parameters for MAXMotion Velocity control
          // CruiseVelocity is not included here as it is specifically called out in the docs to only affect position control
          .maxAcceleration(1000 * TurretUnits.kYawVelocityConversionFactor, ClosedLoopSlot.kSlot1) // degrees per sec/s
          .allowedProfileError(TurretSetpoints.kYawVelocityTolerance, ClosedLoopSlot.kSlot1); // degrees per sec

      // Motor kV is 1/motor free speed rpm. Feedforward config expects this value as a factor of V/rpm so multiply by
      // the nominal voltage
      turretYawConfig.closedLoop
        .feedForward
          .kV(0.05)
          .kS(0.05); // TODO: Update/tune these values later
          //.kV(nominalVoltage / (Constants.Neo550MotorConstants.kFreeSpeedRpm * TurretUnits.kYawVelocityConversionFactor));

      turretPitchConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .openLoopRampRate(1.0)
        .closedLoopRampRate(1.0)
        .smartCurrentLimit(Neo550MotorConstants.kMaxAllowedCurrent - 5, Neo550MotorConstants.kMaxAllowedCurrent)
        .secondaryCurrentLimit(Neo550MotorConstants.kMaxAllowedCurrent, 20);

      // turretPitchConfig
      //   .softLimit
      //     .forwardSoftLimit(TurretSetpoints.kPitchMotorMaxSetpoint)
      //     .forwardSoftLimitEnabled(true)
      //     .reverseSoftLimit(TurretSetpoints.kPitchMotorMinSetpoint)
      //     .reverseSoftLimitEnabled(true);

      turretPitchConfig
        .encoder
          .positionConversionFactor(TurretUnits.kPitchPositionConversionFactor)
          .velocityConversionFactor(TurretUnits.kPitchVelocityConversionFactor);

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
          .cruiseVelocity(1000 * TurretUnits.kPitchVelocityConversionFactor) // degrees per sec
          .maxAcceleration(1000 * TurretUnits.kPitchVelocityConversionFactor) // degrees per sec/s
          .allowedProfileError(TurretSetpoints.kPitchPositionTolerance) // degrees
          // Set MAXMotion parameters for MAXMotion Velocity control
          // CruiseVelocity is not included here as it is specifically called out in the docs to only affect position control
          .maxAcceleration(1000 * TurretUnits.kPitchVelocityConversionFactor, ClosedLoopSlot.kSlot1) // degrees per sec/s
          .allowedProfileError(TurretSetpoints.kPitchVelocityTolerance, ClosedLoopSlot.kSlot1); // degrees per sec

      // Motor kV is 1/motor free speed rpm. Feedforward config expects this value as a factor of V/rpm so multiply by
      // the nominal voltage
      turretPitchConfig.closedLoop
        .feedForward
          .kV(0.05)
          .kS(0.05); // TODO: Update/tune these values later
          //.kV(nominalVoltage / (Constants.Neo550MotorConstants.kFreeSpeedRpm * TurretUnits.kPitchVelocityConversionFactor)); // output degrees per sec

    }
  }

  public static final class FeederSubsystem {
    public static final SparkMaxConfig feederConfig = new SparkMaxConfig();

    static {

      // Configure basic setting of the feeder motor
      feederConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .openLoopRampRate(1.0)
        .smartCurrentLimit(30, 60);

    }
  }

  public static final class ShooterSubsystem {

    public static final SparkMaxConfig flywheelConfig = new SparkMaxConfig();
    public static final SparkMaxConfig flywheelFollowerConfig = new SparkMaxConfig();

    static {
      // Configure basic setting of the flywheel motors
      flywheelConfig
        .inverted(true)
        .idleMode(IdleMode.kCoast)
        .closedLoopRampRate(1.0)
        .openLoopRampRate(1.0)
        .smartCurrentLimit(30, 50);

      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      flywheelConfig
        .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for velocity control
          .p(0.0)
          .outputRange(-1, 1);

      flywheelConfig.closedLoop
        .maxMotion
          // Set MAXMotion parameters for MAXMotion Velocity control
          // CruiseVelocity is not included here as it is specifically called out in the docs to only affect position control
          .maxAcceleration(4000) // rpm/s
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
    }
  }

  public static final class ClimberSubsystem {

    public static TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    public static TalonFXConfiguration rightConfig = new TalonFXConfiguration();

    static {

      // Make sure the motors are using the built-in relative encoder on the motor shaft
      leftConfig
         .Feedback
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

      // Set the motors to brake when not be commanded to move, and set proper rotation
      leftConfig
         .MotorOutput
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive);

      // Clamp the open loop ramp rate
      // TODO: Re-evaluate this later
      leftConfig
         .OpenLoopRamps
            .withDutyCycleOpenLoopRampPeriod(0.5);

      // Set current limits for the motors to prevent damage
      // If the supply current limit is reached for a period of time, we will drop down to the lower limit
      // for the defined time period
      leftConfig
         .CurrentLimits
            .withSupplyCurrentLimit(40)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLowerLimit(30)
            .withSupplyCurrentLowerTime(1.5)
            .withStatorCurrentLimit(60)
            .withStatorCurrentLimitEnable(true); // Amps

      // Apply the same configuration on the right config
      rightConfig = leftConfig.clone();
    }
  }

  public static final class CANdleSubsystem {

    public static final CANdleConfiguration config = new CANdleConfiguration();

    static {
      config
         .LED
            .withStripType(StripTypeValue.GRB)
            .withLossOfSignalBehavior(LossOfSignalBehaviorValue.KeepRunning)
            .withBrightnessScalar(LightsSubsystemConstants.kGlobalBrightnessValue);

      config.
         CANdleFeatures
            .withEnable5VRail(Enable5VRailValue.Enabled)
            .withStatusLedWhenActive(StatusLedWhenActiveValue.Enabled)
            .withVBatOutputMode(VBatOutputModeValue.On);
    }
  }
}
