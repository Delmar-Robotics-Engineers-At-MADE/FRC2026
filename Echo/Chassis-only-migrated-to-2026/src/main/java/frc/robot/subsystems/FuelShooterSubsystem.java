package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Configs;
import frc.robot.Constants.ShooterSubsystemConstants;
import frc.robot.Constants.ShooterSubsystemConstants.FlywheelSetpoints;

public class FuelShooterSubsystem extends SubsystemBase {

   // Flywheel components
   private SparkMax m_motorPort, m_motorStar;
   private SparkClosedLoopController m_flywheelClosedLoopController;
   private RelativeEncoder m_flywheelEncoder;

   private final InterpolatingDoubleTreeMap m_flywheelMap = new InterpolatingDoubleTreeMap();

   // Odometry class for tracking robot pose
   SwerveDrivePoseEstimator m_odometry = null;  // filled in by constructor

   // Member variables for subsystem state management
   double m_flywheelTargetVelocity = ShooterSubsystemConstants.FlywheelSetpoints.kShootRpm;

   private static double m_flywheelSetpointDeadband = 20.0; // RPM

   public FuelShooterSubsystem(SwerveDrivePoseEstimator robot_odometry) {

      // for tracking hub by odometry
      m_odometry = robot_odometry;

      // Initialize flywheel motors
      m_motorPort = new SparkMax(ShooterSubsystemConstants.kFlywheelMotorCanId, MotorType.kBrushless);
      m_motorStar = new SparkMax(ShooterSubsystemConstants.kFlywheelFollowerMotorCanId, MotorType.kBrushless);

      // Flywheel motors are connected together so use the leader's closed loop
      // controller and encoder for control
      m_flywheelClosedLoopController = m_motorPort.getClosedLoopController();
      m_flywheelEncoder = m_motorPort.getEncoder();

      // Key = distance in feet, Value = hood angle in degrees
      m_flywheelMap.put(4.0, 2500.0); //2500
      m_flywheelMap.put(8.0, 2750.0);
      m_flywheelMap.put(12.0, 3250.0);
      m_flywheelMap.put(16.0, 3750.0); // 5000
      m_flywheelMap.put(20.0, 4250.0); // 5000
      m_flywheelMap.put(24.0, 4825.0); // 5000
      m_flywheelMap.put(24.000001, 3500.0); // 5000
      m_flywheelMap.put(43.0, 5000.0); // 5000

      /*
       * Apply the appropriate configurations to the SPARKs.
       *
       * kResetSafeParameters is used to get the SPARK to a known state. This
       * is useful in case the SPARK is replaced.
       *
       * kPersistParameters is used to ensure the configuration is not lost when
       * the SPARK loses power. This is useful for power cycles that may occur
       * mid-operation.
       */
      m_motorPort.configure(
            Configs.ShooterSubsystem.flywheelConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
      m_motorStar.configure(
            Configs.ShooterSubsystem.flywheelFollowerConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

      // Zero encoders on initialization
      m_flywheelEncoder.setPosition(0.0);
   }

   private boolean isFlywheelAt(double velocity) {
      return MathUtil.isNear(m_flywheelEncoder.getVelocity(),
            velocity, FlywheelSetpoints.kVelocityTolerance);
   }

   /**
    * Trigger: Is the flywheel spinning at the required velocity?
    */
   public final Trigger isFlywheelSpinning = new Trigger(
         () -> isFlywheelAt(this.m_flywheelTargetVelocity) ||
               m_flywheelEncoder.getVelocity() > this.m_flywheelTargetVelocity);

   public final Trigger isFlywheelSpinningBackwards = new Trigger(
         () -> isFlywheelAt(-this.m_flywheelTargetVelocity) ||
               m_flywheelEncoder.getVelocity() < -this.m_flywheelTargetVelocity);

   /**
    * Trigger: Is the flywheel stopped?
    */
   public final Trigger isFlywheelStopped = new Trigger(() -> isFlywheelAt(0));

   /**
    * Calculate the target flywheel speed based on the specified target and the robot's curret distance
    * the target
    * @param targetPos The target position to shoot at
    */
   public void calculateFlywheelSpeed(Translation2d targetPos) {

      // calculate angle to red target, and then pretend joystick is pointing that way
      Pose2d pose = m_odometry.getEstimatedPosition();

      // Calculate the rotation the field relative angle to the target from the robot's center
      Translation2d robotPos = pose.getTranslation();

      double newVelocity = m_flywheelMap.get(Units.metersToFeet(targetPos.getDistance(robotPos)));

      // Update the target flywheel velocity
      if (Math.abs(m_flywheelTargetVelocity - newVelocity) > m_flywheelSetpointDeadband)
      {
         m_flywheelTargetVelocity = newVelocity;
      }
   }

   /**
    * Drive the flywheels to their set velocity. This will use MAXMotion
    * velocity control which will allow for a smooth acceleration and deceleration
    * to the mechanism's
    * setpoint.
    */
   private void setFlywheelVelocity(double velocity) {
      m_flywheelClosedLoopController.setSetpoint(velocity, ControlType.kMAXMotionVelocityControl);
   }

   private void setToTargetVelocity() {
      setFlywheelVelocity(m_flywheelTargetVelocity);
   }

   /**
    * Command to run the flywheel motors to track speed of based on its current target distance
    */
   public Command trackHubCommand(Translation2d targetPos) {
      return Commands.run(() -> {
         calculateFlywheelSpeed(targetPos);
         setToTargetVelocity();
      }, this)
      .finallyDo(() -> m_motorPort.stopMotor())
      .withName("Track Hub Flywheel");
   }

   /**
    * Command to run the flywheel motors. When the command is interrupted, e.g. the
    * button is released,
    * the motors will stop.
    */
   public Command runFlywheelCommand() {
      return this.startEnd(
            () -> {
               System.out.println("Spinning Flywheel!!");
               this.setFlywheelVelocity(m_flywheelTargetVelocity);
            },
            () -> {
               m_motorPort.stopMotor();
            }).withName("Spinning Up Flywheel");
   }

   public double getVelocity() {
      return m_flywheelEncoder.getVelocity();
   }

   @Override
   public void periodic() {

      // Flyhweel attributes
      SmartDashboard.putNumber("Flywheel | Temperature (deg C)", m_motorPort.getMotorTemperature());
      SmartDashboard.putNumber("Flywheel | Applied Output", m_motorPort.getAppliedOutput());
      SmartDashboard.putNumber("Flywheel | Current", m_motorPort.getOutputCurrent());
      SmartDashboard.putNumber("Flywheel | Velocity Controller Setpoint", m_flywheelClosedLoopController.getMAXMotionSetpointVelocity());

      // Flywheel follower attributes
      SmartDashboard.putNumber("Flywheel Follower | Temperature (deg C)", m_motorStar.getMotorTemperature());
      SmartDashboard.putNumber("Flywheel Follower | Applied Output", m_motorStar.getAppliedOutput());
      SmartDashboard.putNumber("Flywheel Follower | Current", m_motorStar.getOutputCurrent());

      // Target velocity vs actual velocity
      SmartDashboard.putNumber("Flywheel | Target Velocity", m_flywheelTargetVelocity);
      SmartDashboard.putNumber("Flywheel | Actual Velocity", m_flywheelEncoder.getVelocity());

      // Track whether the flywheel is spinning (within the tolerance)
      SmartDashboard.putBoolean("Is Flywheel Spinning", isFlywheelSpinning.getAsBoolean());
      SmartDashboard.putBoolean("Is Flywheel Stopped", isFlywheelStopped.getAsBoolean());
   }
}
