package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.Neo550MotorConstants;
import frc.robot.Constants.TurretSubsystemConstants;
import frc.robot.Constants.TurretSubsystemConstants.TurretSetpoints;
import frc.robot.Constants.TurretSubsystemConstants.TurretUnits;

public class TurretSubsystem extends SubsystemBase {

   // Turret components
   private SparkMax m_turretYawMotor, m_turretPitchMotor; // rotation and hood control motors
   private SparkClosedLoopController m_turretYawClosedLoopController, m_turretPitchClosedLoopController;
   private RelativeEncoder m_turretYawEncoder, m_turretPitchEncoder;

   // Sensors
   private final DigitalInput m_hallEffectYaw = new DigitalInput(0);

   private boolean m_isTurretYawHomed = false;
   private boolean m_isTurretPitchHomed = false;

   private double m_turretPitchkG = 1.0;
   private double m_turretYawkS = 0.0;

   // REMOVE LATER: Tuning Constants
   private SparkMaxConfig mt_turretConfig = Configs.TurretSubsystem.turretYawConfig;
   private double mt_turretClosedLoopP = 0.0;
   private double mt_turretClosedLoopI = 0.0;
   private double mt_turretClosedLoopD = 0.0;

   // Odometry class for tracking robot pose
   SwerveDrivePoseEstimator m_odometry = null;  // filled in by constructor

   private Translation2d m_hub;

   public TurretSubsystem(SwerveDrivePoseEstimator robot_odometry) {

     // for tracking hub by odometry
      m_odometry = robot_odometry;

      Optional<Alliance> allianceOptional = DriverStation.getAlliance();
      // target position on field
      if(allianceOptional.isPresent() && allianceOptional.get() == Alliance.Red) {
         m_hub = new Translation2d(11.92, 4.03);
      } else {
         m_hub = new Translation2d(4.63, 4.03);
      }


      // Initialize shooter pointing motors (yaw motor controls the shooter's
      // direction while the pitch motor controls hood position)
      m_turretYawMotor = new SparkMax(TurretSubsystemConstants.kTurretYawMotorCanId, MotorType.kBrushless);
      m_turretPitchMotor = new SparkMax(TurretSubsystemConstants.kTurretPitchMotorCanId, MotorType.kBrushless);

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
      m_turretYawMotor.configure(
            Configs.TurretSubsystem.turretYawConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
      m_turretPitchMotor.configure(
            Configs.TurretSubsystem.turretPitchConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);


      // Initialize individual closed loop controllers and motors for each of the
      // turret components individually
      m_turretYawClosedLoopController = m_turretYawMotor.getClosedLoopController();
      m_turretYawEncoder = m_turretYawMotor.getEncoder();
      m_turretPitchClosedLoopController = m_turretPitchMotor.getClosedLoopController();
      m_turretPitchEncoder = m_turretPitchMotor.getEncoder();

      // Zero encoders on initialization
      m_turretYawEncoder.setPosition(0.0);
      m_turretPitchEncoder.setPosition(0.0);

      // TODO: Remove these later; tuning constants
      SmartDashboard.putNumber("Set Turret/kP", mt_turretClosedLoopP);
      SmartDashboard.putNumber("Set Turret/kI", mt_turretClosedLoopI);
      SmartDashboard.putNumber("Set Turret/kD", mt_turretClosedLoopD);
      SmartDashboard.putNumber("Set Turret Pitch kG", m_turretPitchkG);
      SmartDashboard.putNumber("Set Turret Yaw kS", m_turretYawkS);
   }

   // TODO: Move most of this logic to a separate class/subsystem for getting the angle to a particular target
   // Need to decide if this class should be handling conversion from field-relative to robot-relative/turret relative
    public void trackHub () {

      // calculate angle to red target, and then pretend joystick is pointing that way
      Pose2d pose = m_odometry.getEstimatedPosition();

      // Calculate the rotation the field relative angle to the target from the robot's center
      Translation2d robotPos = pose.getTranslation();
      Rotation2d fieldRelativeAngleToTarget = m_hub.minus(robotPos).getAngle();

      // Get the current robot heading and subtract it from the field-relative number to get the
      // angle relative to the robot's local reference frame (where 0 degrees is straight forward)
      // This provides a valid in the range [-180, 180]
      Rotation2d robotHeading = pose.getRotation();
      double robotRelativeDeg = fieldRelativeAngleToTarget.minus(robotHeading).getDegrees();

      // Add the offset to the output value to get the angle representing straight forward on the turret
      double turretSetpointDeg = robotRelativeDeg + TurretSetpoints.kYawCenterOffsetFromHome;

      // Normalize the value in degrees so it falls in the range [0,360]
      turretSetpointDeg = ((turretSetpointDeg % 360.0) + 360.0) % 360.0;

      moveTurretYawToPosition(turretSetpointDeg);
   }
       
   /**
    * Helper function to determining if the turret's rotation is within a small margin of error from the desired position
    * @param desiredPosition The position to compare against the actual position
    * @return Whether the current position of the turret is "close enough" to the desired position
    */
   private boolean isYawAt(double desiredPosition) {
      return Math.abs(m_turretYawEncoder.getPosition() - desiredPosition) < TurretSetpoints.kYawPositionTolerance;
   }

   /**
    * Checks whether the input position falls within the valid range of positions that the turret can be commanded to
    * @param position The input position to verify against the turret bounds
    * @return Whether the turret position is a valid setpoint
    */
   private boolean isYawPositionReachable(double position) {
      return position >= TurretSetpoints.kYawMotorMinSetpoint && position <= TurretSetpoints.kYawMotorMaxSetpoint;
   }

   /**
    * Helper function to determining if the turret's hood is within a small margin of error from the desired position
    * @param desiredPosition The position to compare against the actual position
    * @return Whether the current position of the hood is "close enough" to the desired position
    */
   private boolean isPitchAt(double desiredPosition) {
      return Math.abs(m_turretPitchEncoder.getPosition() - desiredPosition) < TurretSetpoints.kPitchPositionTolerance;
   }

   /**
    * Trigger: Is the turret at the desired position?
    */
   public final BooleanSupplier isYawAtPosition(double position) {
      return () -> isYawAt(position);
   }

   /**
    * Trigger: Is the turret hood at the desired position?
    */
   public final BooleanSupplier isPitchAtPosition(double position) {
      return () -> isPitchAt(position);
   }

   /**
    * Used for testing and backup ability for manual control of the turret's
    * rotation
    * 
    * @param dutyCycle A value from [-1, 1]
    */
   private void moveTurretYaw(double dutyCycle) {

      System.out.println("MoveTurretYaw");
      // Clamp the applied duty cycle to 30% for safety in testing
      double actualAppliedDutyCycle = Math.max(-0.3, Math.min(0.3, dutyCycle));
      m_turretYawMotor.set(actualAppliedDutyCycle);
   }

   public void moveTurretYawVelocity(double velocity) {
      m_turretYawClosedLoopController.setSetpoint(velocity, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot1);
   }

   private void moveTurretPitchVelocity(double velocity) {
      m_turretPitchClosedLoopController.setSetpoint(velocity, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot1);
   }

   /**
    * Used to command the turret's yaw motor to a particular position in
    * degrees relative to the turret's local reference frame in the range [0,300]
    * 
    * @param position Actual output position of the turret's rotation in degrees in the range [0,300]
    */
   private void moveTurretYawToPosition(double position) {
      if (isTurretYawHomed().getAsBoolean()) {

         // Clamp the value so we don't move past the safe boundaries
         double actualAppliedPosition = Math.max(TurretSetpoints.kYawMotorMinSetpoint, Math.min(TurretSetpoints.kYawMotorMaxSetpoint, position));
         m_turretYawClosedLoopController.setSetpoint(actualAppliedPosition, ControlType.kMAXMotionPositionControl);
      }
   }

   /**
    * Used to return whether or not the yaw turret motor has been homed and is
    * ready to be commaned to a particular position
    * This function must always be return true before utilizing closed-loop
    * position control on the yaw motor
    * 
    * @return Whether the yaw turret has been homed
    */
   public BooleanSupplier isTurretYawHomed() {
      return () -> m_isTurretYawHomed;
   }

   /**
    * Gets the current state of the sensor being used to determine if the yaw
    * turret is at its homed position
    * This can include any failsafes that could be implented to protect against a
    * failing sensor
    * 
    * @return A boolean indicating if the the yaw motor has reached its homing
    *         setpoint
    */
   private BooleanSupplier getTurretYawAtHome() {
      return () -> !m_hallEffectYaw.get() || 
                   ((m_turretYawMotor.getOutputCurrent() > Neo550MotorConstants.kMaxAllowedCurrent / 3) && 
                   Math.abs(m_turretYawEncoder.getVelocity()) < TurretUnits.kTurretYawNotMovingSafeThresholdDegreesPerSec);
   }

   /**
    * Used to declare that the yaw turret has been homed and its absolute position
    * can be set according to hard physical limits
    * This function is intended to be run at the beginning of autonomous init in
    * order to get the turret's absolute output rotation
    */
   private void setTurretYawHomed() {
      m_isTurretYawHomed = true;
      m_turretYawEncoder.setPosition(TurretSetpoints.kYawMotorHomingSetpoint);

      // TODO: Add logic here to enable soft limits in the motor controller
   }

   /**
    * Used for testing and backup ability for manual control of the turret's hood
    * position
    * 
    * @param dutyCycle A value from [-1, 1]
    */
   private void moveTurretPitch(double dutyCycle) {

      // Clamp the applied duty cycle to 30% for safety in testing
      double actualAppliedDutyCycle = Math.max(-0.3, Math.min(0.3, dutyCycle));
      m_turretPitchMotor.set(actualAppliedDutyCycle);
   }

   /**
    * Used to command the turret's hood to a particular absolute position in
    * degrees
    * 
    * @param positionDegrees Absolute output position of the turret's hood in degrees
    *                 (launch angle)
    */
   private void moveTurretPitchToPosition(double positionDegrees) {
      if (isTurretPitchHomed().getAsBoolean()) {

         // Clamp the value so we don't move past the safe boundaries
         double actualAppliedPosition = Math.max(TurretSetpoints.kPitchMotorMinSetpoint, Math.min(TurretSetpoints.kPitchMotorMaxSetpoint, positionDegrees));

         double arbFF = m_turretPitchkG * Math.sin(Math.toRadians(positionDegrees));
         m_turretPitchClosedLoopController.setSetpoint(actualAppliedPosition, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, arbFF);
      }
   }

   /**
    * Used to return whether or not the pitch turret motor has been homed and is
    * ready to be commaned to a particular position
    * This function must always be return true before utilizing closed-loop
    * position control on the pitch motor
    * 
    * @return Whether the pitch turret has been homed
    */
   private BooleanSupplier isTurretPitchHomed() {
      return () -> m_isTurretPitchHomed;
   }

   /**
    * Gets the current state of the sensor being used to determine if the pitch
    * turret is at its homed position
    * This can include any failsafes that could be implented to protect against a
    * failing sensor
    * 
    * @return A boolean indicating if the the pitch motor has reached its homing
    *         setpoint
    */
   private BooleanSupplier getTurretPitchAtHome() {
      return () -> (m_turretPitchMotor.getOutputCurrent() > Neo550MotorConstants.kMaxAllowedCurrent / 3) && 
                   Math.abs(m_turretPitchEncoder.getVelocity()) < TurretUnits.kTurretPitchNotMovingSafeThresholdDegreesPerSec;
   }

   /**
    * Used to declare that the pitch turret has been homed and its absolute
    * position can be set according to hard physical limits
    * This function is intended to be run at the beginning of autonomous init in
    * order to get the hood's absolute output rotation
    */
   private void setTurretPitchHomed() {
      m_isTurretPitchHomed = true;
      m_turretPitchEncoder.setPosition(TurretSetpoints.kPitchMotorHomingSetpoint);

      // TODO: Add logic here to enable soft limits in the motor controller
   }

   // TODO: remove this later!!
   private void testSetTurretHomed() {
      setTurretPitchHomed();
      setTurretYawHomed();
   }

      /**
    * Command to manually home the turret's rotation. While being commanded, the turret
    * will be allowed to move via manual stick motion until the hall effect sensor is reached
    */
   public Command homeTurretYaw(DoubleSupplier axisSupplier) {
      return Commands.run(
               () -> moveTurretYaw(axisSupplier.getAsDouble() * TurretSubsystemConstants.kTurretYawManualHomeDutyCycle), 
               this)
            .until(getTurretYawAtHome())
            .andThen(Commands.runOnce(() -> {
               m_turretYawMotor.stopMotor();
               setTurretYawHomed();
            }, this))
            .withName("Manual Turret Homing");
   }

   /**
    * Commands the turret to rotate to an absolute position
    * 
    * @param positionDegrees The desired absolute rotational position of the turret
    */
   public Command commandTurretYawToPosition(DoubleSupplier positionDegrees) {
      return this.startEnd(
            () -> {
               this.moveTurretYawToPosition(positionDegrees.getAsDouble());
            }, () -> {
               this.m_turretYawMotor.stopMotor();
            }).withName("Rotating turret yaw to position");
   }

   /**
    * Command to stop the turret from rotating in general and to fall back to its brake mode to hold its current position
    */
   public Command stopTurretYaw() {
      return this.run(() -> m_turretYawMotor.stopMotor());
   }

   /**
    * Command to home the turret yaw motor at a low duty cycle and set its starting
    * parameters to allow closed loop position control
    */
   public Command homeTurretPitch() {
      return run(() -> this.moveTurretPitch(0.1))
            .until(getTurretPitchAtHome())
            .andThen(() -> this.m_turretPitchMotor.stopMotor())
            .andThen(() -> this.setTurretPitchHomed())
            .withName("Home Turret Pitch");
   }

   /**
    * Commands the turret hood to move to an absolute position
    * 
    * @param position The desired absolute position of the turret hood
    */
   public Command commandTurretPitchToPosition(DoubleSupplier position) {
      return this.startEnd(
            () -> {
               this.moveTurretPitchToPosition(position.getAsDouble());
            }, () -> {
               this.m_turretPitchMotor.stopMotor();
            }).withName("Rotating turret pitch to position");
   }

   public Command testCommandSetTurretHomed() {
      return this.runOnce(() -> this.testSetTurretHomed());
   }

   // TODO: Remove this or move it to a shared place later. There is a matching function in the shooter class
   private boolean hasChanged(double a, double b) {
      return Math.abs(a - b) > 1e-6;
   }

   @Override
   public void periodic() {

      // Display subsystem values
      SmartDashboard.putNumber("Turret/Position (deg)", m_turretYawEncoder.getPosition());
      SmartDashboard.putNumber("Turret/Current (A)", m_turretYawMotor.getOutputCurrent());
      SmartDashboard.putNumber("Turret/Temperature (deg C)", m_turretYawMotor.getMotorTemperature());
      
      SmartDashboard.putNumber("Hood/Position (deg)", m_turretPitchEncoder.getPosition());
      SmartDashboard.putNumber("Hood/Current (A)", m_turretPitchMotor.getOutputCurrent());
      SmartDashboard.putNumber("Hood/Temperature (deg C)", m_turretPitchMotor.getMotorTemperature());

      m_turretPitchkG = SmartDashboard.getNumber("Set Turret Pitch kG", m_turretPitchkG);
      double newTurretYawkS = SmartDashboard.getNumber("Set Turret Yaw kS", m_turretYawkS);

      // Sensors
      SmartDashboard.putBoolean("Hall Effect Sensor Detection", !m_hallEffectYaw.get());

      // REMOVE LATER: Tuning PID for the flywheel
      double newTurretkP = SmartDashboard.getNumber("Set Turret/kP", mt_turretClosedLoopP);
      double newTurretkI = SmartDashboard.getNumber("Set Turret/kI", mt_turretClosedLoopI);
      double newTurretkD = SmartDashboard.getNumber("Set Turret/kD", mt_turretClosedLoopD);

      if (hasChanged(newTurretkP, mt_turretClosedLoopP)|| hasChanged(newTurretkI, mt_turretClosedLoopI) || hasChanged(newTurretkD, mt_turretClosedLoopD)) {
         mt_turretConfig
            .closedLoop
               .p(newTurretkP)
               .i(newTurretkI)
               .d(newTurretkD)
               .p(newTurretkP, ClosedLoopSlot.kSlot2)
               .i(newTurretkI, ClosedLoopSlot.kSlot2)
               .d(newTurretkD, ClosedLoopSlot.kSlot2);

         m_turretYawMotor.configure(mt_turretConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

         mt_turretClosedLoopP = newTurretkP;
         mt_turretClosedLoopI = newTurretkI;
         mt_turretClosedLoopD = newTurretkD;
      }

      if (hasChanged(m_turretYawkS, newTurretYawkS))
      {
         // Configure only closed loop slot 2 with the kS constant since it is working against the energy chain and constant force spring
         mt_turretConfig
            .closedLoop
               .feedForward
                  .kS(newTurretYawkS, ClosedLoopSlot.kSlot2);

         m_turretYawMotor.configure(mt_turretConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

         m_turretYawkS = newTurretYawkS;
      }

      // Push current values so they appear on startup
      SmartDashboard.putNumber("Turret/kP", mt_turretClosedLoopP);
      SmartDashboard.putNumber("Turret/kI", mt_turretClosedLoopI);
      SmartDashboard.putNumber("Turret/kD", mt_turretClosedLoopD);
   }
}
