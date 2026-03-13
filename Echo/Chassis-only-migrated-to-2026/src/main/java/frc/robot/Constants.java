// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

   public static final class IntakeSubsystemConstants {
      public static final int kIntakeMotorCanId = 33; // SPARK MAX CAN ID
      public static final int kConveyorMotorCanId = 32; // SPARK MAX CAN ID

      public static final class IntakeSetpoints {
         public static final double kIntake = 0.6;
         public static final double kExtake = -0.6;
      }

      public static final class ConveyorSetpoints {
         public static final double kIntake = 0.3;
         public static final double kExtake = -0.3;
      }
   }

   public static final class FeederSubsystemConstants {
      public static final int kFeederMotorCanId = 31; // SPARK MAX CAN ID

      public static final class FeederSetpoints {
         public static final double kFeed = 0.6; // duty cycle
      }
   }

   public static final class TurretSubsystemConstants {
      public static final int kTurretYawMotorCanId = 24;
      public static final int kTurretPitchMotorCanId = 23;

      public static final class TurretUnits {
         private static final double kYawMotorOutputGearRatio = 62.24615385; // (119 * 68:13) / 10
         public static final double kYawPositionConversionFactor = 360.0 / kYawMotorOutputGearRatio; // degrees
         public static final double kYawVelocityConversionFactor = 360.0 / (60.0 * kYawMotorOutputGearRatio); // degrees per second

         private static final double kPitchMotorOutputGearRatio = 361.398601399; // (68 * 76 * 28 * 150) / (13 * 21 * 22 * 10)
         public static final double kPitchPositionConversionFactor = 360.0 / kPitchMotorOutputGearRatio; // degrees
         public static final double kPitchVelocityConversionFactor = 360.0 / (60.0 * kPitchMotorOutputGearRatio); // degrees per second

         public static final double kTurretYawNotMovingSafeThresholdDegreesPerSec = 5.0 * kYawVelocityConversionFactor; // 5 RPM with conversion factor applied
         public static final double kTurretPitchNotMovingSafeThresholdDegreesPerSec = 5.0 * kPitchVelocityConversionFactor; // 5 RPM with conversion factor applied
      }

      public static final class TurretSetpoints {
         public static final double kYawPositionTolerance = 1.0; // degrees; TODO: Adjust this later
         public static final double kPitchPositionTolerance = 1.0; // degrees; TODO: Adjust this later

         public static final double kYawVelocityTolerance = 1; // degrees per second
         public static final double kPitchVelocityTolerance = 1; // degrees per second

         public static final double kYawMotorHomingSetpoint = 60.0; // degrees
         public static final double kPitchMotorHomingSetpoint = 28.0; // degrees

         public static final double kYawOutputRotationSafeBuffer = 2.0; // 2 degrees
         public static final double kPitchOutputAngleSafeBuffer = 5.0; // 5 degrees; TODO: extra large for now to make sure we have a safe buffer

         public static final double kYawMotorMinSetpoint = 0.0 + kYawOutputRotationSafeBuffer; // 2 degrees; the hard stop is at 0 degrees so add a couple extra degrees of buffer
         public static final double kYawMotorMaxSetpoint = 300.0 - kYawOutputRotationSafeBuffer; // 300 degrees with a safe buffer; hard stop at 300 degrees so remove a couple degrees for a safety net
         public static final double kPitchMotorMinSetpoint = kPitchMotorHomingSetpoint + kPitchOutputAngleSafeBuffer; // use the  homing setpoint as the min value and remove a couple degrees for a safety net
         public static final double kPitchMotorMaxSetpoint = 68.0 - kPitchOutputAngleSafeBuffer; // 28 degrees with a safe buffer; hard stop at 28 degrees so remove a couple degrees for a safety net
      }
   }

   public static final class ShooterSubsystemConstants {
      public static final int kFlywheelMotorCanId = 22; // SPARK MAX CAN ID (Port)
      public static final int kFlywheelFollowerMotorCanId = 21; // SPARK MAX CAN ID (Star)

      public static final class FlywheelSetpoints {
         public static final double kShootRpm = 4500.0;
         public static final double kVelocityTolerance = 250.0; // rpm
      }
   }

   public static final class ClimberSubsystemConstants {
      public static final int kClimberLeftMotorCanId = 41; // CTRE Talon FX CAN ID (port-side motor)
      public static final int kClimberRightMotorCanId = 42; // CTRE Talon FX CAN ID (star-side motor)
   }

   public static final class LightsSubsystemConstants {
      public static final int kCANdleCanId = 51; // CTRE CANdle CAN ID
      public static final double kGlobalBrightnessValue = 0.5; // 50% of max brightness
   }

   public static final class DriveConstants {
      // Driving Parameters - Note that these are not the maximum capable speeds of
      // the robot, rather the allowed maximum speeds
      public static final double kMaxSpeedMetersPerSecond = 4.8;
      public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

      // #####################
      // Chassis Configuration
      // #####################

      // Distance between centers of right and left wheels on robot
      public static final double kTrackWidth = Units.inchesToMeters(23.5);

      // Distance between front and back wheels on robot
      public static final double kWheelBase = Units.inchesToMeters(23.5);

      public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

      // Angular offsets of the modules relative to the chassis in radians
      public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
      public static final double kFrontRightChassisAngularOffset = 0;
      public static final double kBackLeftChassisAngularOffset = Math.PI;
      public static final double kBackRightChassisAngularOffset = Math.PI / 2;

      // SPARK MAX CAN IDs
      public static final int kFrontLeftDrivingCanId = 12;
      public static final int kRearLeftDrivingCanId = 13;
      public static final int kFrontRightDrivingCanId = 11;
      public static final int kRearRightDrivingCanId = 14;

      public static final int kFrontLeftTurningCanId = 2;
      public static final int kRearLeftTurningCanId = 3;
      public static final int kFrontRightTurningCanId = 1;
      public static final int kRearRightTurningCanId = 4;

      public static final boolean kGyroReversed = true;
   }

   public static final class ModuleConstants {
      // The MAXSwerve module can be configured with one of three pinion gears: 12T,
      // 13T, or 14T.
      // This changes the drive speed of the module (a pinion gear with more teeth
      // will result in a
      // robot that drives faster).
      public static final int kDrivingMotorPinionTeeth = 12;

      // Invert the turning encoder, since the output shaft rotates in the opposite
      // direction of
      // the steering motor in the MAXSwerve Module.
      // public static final boolean kTurningEncoderInverted = true;
      // The above is done now in the Rev Hardware Client when zeroing the encoder

      // Calculations required for driving motor conversion factors and feed forward
      public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
      public static final double kWheelDiameterMeters = 0.0762;
      public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
      // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
      // teeth on the bevel pinion
      public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
      public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
            / kDrivingMotorReduction;

      // These are in Configs.java now, was...
      // public static final double kDrivingP = 0.04;
      // public static final double kDrivingI = 0;
      // public static final double kDrivingD = 0;
      // public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
      // public static final double kDrivingMinOutput = -1;
      // public static final double kDrivingMaxOutput = 1;
   }

   public static final class OIConstants {
      public static final int kDriverControllerPort = 0;
      public static final int kOperatorControllerPort = 1;
      public static final int kButtonPadPort = 2;
      public static final double kDriveDeadband = 0.05;
   }

   public static final class AutoConstants {
      public static final double kMaxSpeedMetersPerSecond = 3;
      public static final double kMaxAccelerationMetersPerSecondSquared = 3;
      public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
      public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

      public static final double kPXController = 1;
      public static final double kPYController = 1;
      public static final double kPThetaController = 1;
      public static final double kTolerance = 0.3; // position control, meters

      // Constraint for the motion profiled robot angle controller
      public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
   }

   public static final class NeoMotorConstants {
      public static final double kFreeSpeedRpm = 5676.0;
   }

   public static final class Neo550MotorConstants {
      public static final double kFreeSpeedRpm = 11000.0;
      public static final int kMaxAllowedCurrent = 20; // Amps
   }

   public static final class LEDConstants {
      public static final double green = 0.77;
      public static final double purple = 0.91;
      public static final double red = -0.31;
      public static final double blue = -0.29;
      public static final double grey = -0.33;
   }
}
