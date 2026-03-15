package frc.robot.subsystems;

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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Configs;
import frc.robot.Constants.DriveConstants;
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

   private double m_turretYawSetpointDegrees = 0.0;
   private boolean m_isTurretYawHomed = false;

   private double m_turretPitchkG = 0.2;
   private double m_turretPitchSetpointDegrees = 0.0;
   private boolean m_isTurretPitchHomed = false;

   // REMOVE LATER: Tuning Constants
   private SparkMaxConfig mt_turretConfig = Configs.TurretSubsystem.turretPitchConfig;
   private double mt_turretClosedLoopP = 0.0;
   private double mt_turretClosedLoopI = 0.0;
   private double mt_turretClosedLoopD = 0.0;

   // Odometry class for tracking robot pose
   SwerveDrivePoseEstimator m_odometry = null;  // filled in by constructor
   private double m_hubX = 0;
   private double m_hubY = 0;

   public TurretSubsystem(SwerveDrivePoseEstimator robot_odometry) {

      // for tracking hub by odometry
      m_odometry = robot_odometry;
      // target position on field
      m_hubY = 4.03;
      if(DriverStation.getAlliance().get() == Alliance.Red) {
         m_hubX = 11.92;
      } else {
         m_hubX = 4.63;
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

      // TODO: REMOVE LATER: Tuning PID for the flywheel
      SmartDashboard.putNumber("Set Turret Yaw Position", m_turretYawSetpointDegrees);
      SmartDashboard.putNumber("Set Turret Pitch Position", m_turretPitchSetpointDegrees);
      SmartDashboard.putNumber("Set Turret/kP", mt_turretClosedLoopP);
      SmartDashboard.putNumber("Set Turret/kI", mt_turretClosedLoopI);
      SmartDashboard.putNumber("Set Turret/kD", mt_turretClosedLoopD);
      SmartDashboard.putNumber("Set Turret Pitch kG", m_turretPitchkG);
   }

   public void trackHub () {

      // calculate angle to red target, and then pretend joystick is pointing that way
      Pose2d pose = m_odometry.getEstimatedPosition();
      double deltaX = m_hubX - pose.getX();
      double deltaY = m_hubY - pose.getY();

      // normalize so one is 1 and the other is < 1
      double maxxy = Math.max(Math.abs(deltaX), Math.abs(deltaY));
      double xSpeed = deltaX / maxxy;
      double ySpeed = deltaY / maxxy;

      System.out.println("trackHub" + xSpeed + " " + ySpeed);
      rotateByOdometry(xSpeed, ySpeed);
   }
   
   public void rotateByOdometry(double xComponent, double yComponent) {
      // Convert the commanded speeds/components into the correct units for the shooter

      var swerveModuleStates = TurretSubsystemConstants.kDriveKinematics.toSwerveModuleStates(
         ChassisSpeeds.fromFieldRelativeSpeeds(xComponent, yComponent, 0,
                  m_odometry.getEstimatedPosition().getRotation()));
      // SwerveDriveKinematics.desaturateWheelSpeeds(
      //     swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
      setDesiredState(swerveModuleStates[0]);
   }   

   public void setDesiredState(SwerveModuleState desiredState) {
      // Apply chassis angular offset to the desired state.
      SwerveModuleState correctedDesiredState = new SwerveModuleState();
      correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(TurretSubsystemConstants.kChassisAngularOffset));

      // Optimize the reference state to avoid spinning further than 90 degrees.
      // No, instead, just change this function to prevent turret from turning through its stops
      // correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));
      correctedDesiredState = optimize(correctedDesiredState);

      // Command driving and turning SPARKS towards their respective setpoints.
      System.err.println("Setting turret setpoint to " + correctedDesiredState.angle.getRadians());
      m_turretYawClosedLoopController.setSetpoint(correctedDesiredState.angle.getDegrees(), ControlType.kPosition);
   }

   private SwerveModuleState optimize(SwerveModuleState state) {
   // original function allows module to always turn less than 180 degrees
   // var delta = angle.minus(currentAngle);
   // if (Math.abs(delta.getDegrees()) > 90.0) {
   //     speedMetersPerSecond *= -1;
   //     angle = angle.rotateBy(Rotation2d.kPi);
   // }
   
   // for the turret, simply stop the turret from moving through the stops
      SwerveModuleState result = state; // return unmodified state if it's not past stops
      if (state.angle.getDegrees() > TurretSubsystemConstants.TurnLimitPort) {
         result = new SwerveModuleState(state.speedMetersPerSecond, 
                                       Rotation2d.fromDegrees(TurretSubsystemConstants.TurnLimitPort));
      } else if (state.angle.getDegrees() < TurretSubsystemConstants.TurnLimitStar) {
         result = new SwerveModuleState(state.speedMetersPerSecond, 
                                       Rotation2d.fromDegrees(TurretSubsystemConstants.TurnLimitStar));
      }
      return result;
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
   public final Trigger isYawAtPosition = new Trigger(
         () -> isYawAt(this.m_turretYawSetpointDegrees));

   /**
    * Trigger: Is the turret hood at the desired position?
    */
   public final Trigger isPitchAtPosition = new Trigger(
         () -> isPitchAt(this.m_turretPitchSetpointDegrees));

   /**
    * Used for testing and backup ability for manual control of the turret's
    * rotation
    * 
    * @param dutyCycle A value from [-1, 1]
    */
   public void moveTurretYaw(double dutyCycle) {

      System.out.println("MoveTurretYaw");
      // Clamp the applied duty cycle to 30% for safety in testing
      double actualAppliedDutyCycle = Math.max(-0.3, Math.min(0.3, dutyCycle));
      m_turretYawMotor.set(actualAppliedDutyCycle);
   }

   public void moveTurretYawVelocity(double velocity) {

      // TODO: Update this to pass in a proper value; for now, use 2 degrees per second
      m_turretYawClosedLoopController.setSetpoint(velocity, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot1);
   }

   private void moveTurretPitchVelocity(double velocity) {

      // TODO: Update this to pass in a proper value; for now, use 2 degrees per second
      m_turretPitchClosedLoopController.setSetpoint(velocity, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot1);
   }

   /**
    * Used to command the turret's yaw motor to a particular absolute position in
    * degrees
    * 
    * @param position Absolute output position of the turret's rotation in degrees
    */
   private void moveTurretYawToPosition(double position) {
      if (isTurretYawHomed()) {

         // Clamp the value so we don't move past the safe boundaries
         // TODO: Update this call to use the position that is being passed in after
         double actualAppliedPosition = Math.max(TurretSetpoints.kYawMotorMinSetpoint, Math.min(TurretSetpoints.kYawMotorMaxSetpoint, m_turretYawSetpointDegrees));
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
   public boolean isTurretYawHomed() {
      return m_isTurretYawHomed;
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
   public BooleanSupplier getTurretYawAtHome() {
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
   public void setTurretYawHomed() {
      m_isTurretYawHomed = true;
      m_turretYawEncoder.setPosition(TurretSetpoints.kYawMotorHomingSetpoint);
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
    * @param position Absolute output position of the turret's hood in degrees
    *                 (launch angle)
    */
   private void moveTurretPitchToPosition() {
      if (isTurretPitchHomed()) {

         // Clamp the value so we don't move past the safe boundaries
         // TODO: Update this call to use the position that is being passed in after
         double actualAppliedPosition = Math.max(TurretSetpoints.kPitchMotorMinSetpoint, Math.min(TurretSetpoints.kPitchMotorMaxSetpoint, m_turretPitchSetpointDegrees));

         double arbFF = m_turretPitchkG * Math.sin(Math.toRadians(m_turretPitchSetpointDegrees));
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
   private boolean isTurretPitchHomed() {
      return m_isTurretPitchHomed;
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
   }

   // TODO: remove this later!!
   private void testSetTurretHomed() {
      setTurretPitchHomed();
      setTurretYawHomed();
   }

      /**
    * Command to manually control the turret's rotation. While being commanded, the
    * turret will move with the
    * applied velocity. Once the command ends, the motors will stop.
    */
   public Command moveTurretRotationManual(DoubleSupplier velocity) {
      return this.startEnd(
            () -> {
               this.moveTurretYawVelocity(velocity.getAsDouble());
            }, () -> {
               this.m_turretYawMotor.stopMotor();
            }).withName("Turning turret");
   }

   /**
    * Command to home the turret yaw motor at a low duty cycle and set its starting
    * parameters to allow closed loop position control
    */
   public Command homeTurretYaw() {
      return run(() -> this.moveTurretYaw(0.1))
            .until(getTurretYawAtHome())
            .andThen(() -> this.m_turretYawMotor.stopMotor())
            .andThen(() -> this.setTurretYawHomed())
            .withName("Home Turret Yaw");
   }

   /**
    * Commands the turret to rotate to an absolute position
    * 
    * @param position The desired absolute rotational position of the turret
    */
   public Command commandTurretYawToPosition(double position) {
      return this.startEnd(
            () -> {
               this.moveTurretYawToPosition(position);
            }, () -> {
               this.m_turretYawMotor.stopMotor();
            }).withName("Rotating turret yaw to position");
   }

   /**
    * Command to manually control the turret's hood. While being commanded, the
    * turret hood will move with the
    * applied velocity. Once the command ends, the motors will stop.
    */
   public Command moveTurretHoodManual(DoubleSupplier velocity) {
      return this.startEnd(
            () -> {
               this.moveTurretPitchVelocity(velocity.getAsDouble());
            }, () -> {
               this.m_turretPitchMotor.stopMotor();
            }).withName("Moving turret hood");
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
   public Command commandTurretPitchToPosition(double position) {
      return this.startEnd(
            () -> {
               this.moveTurretPitchToPosition();
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
      SmartDashboard.putNumber("Turret Yaw | Velocity Setpoint", m_turretYawClosedLoopController.getMAXMotionSetpointVelocity());
      SmartDashboard.putNumber("Turret Yaw | Current", m_turretYawMotor.getOutputCurrent());
      SmartDashboard.putNumber("Turret Pitch | Velocity Setpoint", m_turretPitchClosedLoopController.getMAXMotionSetpointVelocity());
      SmartDashboard.putNumber("Turret Pitch | Current", m_turretPitchMotor.getOutputCurrent());

      // Temps
      SmartDashboard.putNumber("Turret Yaw | Temperature (deg C)", m_turretYawMotor.getMotorTemperature());
      SmartDashboard.putNumber("Turret Pitch | Temperature (deg C)", m_turretPitchMotor.getMotorTemperature());

      SmartDashboard.putNumber("Turret Yaw | Encoder Position", m_turretYawEncoder.getPosition());
      SmartDashboard.putNumber("Turret Pitch | Encoder Position", m_turretPitchEncoder.getPosition());

      m_turretYawSetpointDegrees = SmartDashboard.getNumber("Set Turret Yaw Position", m_turretYawSetpointDegrees);
      m_turretPitchSetpointDegrees = SmartDashboard.getNumber("Set Turret Pitch Position", m_turretPitchSetpointDegrees);
      m_turretPitchkG = SmartDashboard.getNumber("Set Turret Pitch kFF", m_turretPitchkG);

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
               .d(newTurretkD);

         m_turretPitchMotor.configure(mt_turretConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

         mt_turretClosedLoopP = newTurretkP;
         mt_turretClosedLoopI = newTurretkI;
         mt_turretClosedLoopD = newTurretkD;
      }

      // Push current values so they appear on startup
      SmartDashboard.putNumber("Turret/kP", mt_turretClosedLoopP);
      SmartDashboard.putNumber("Turret/kI", mt_turretClosedLoopI);
      SmartDashboard.putNumber("Turret/kD", mt_turretClosedLoopD);
      SmartDashboard.putNumber("Turret Yaw Position Setpoint", m_turretYawSetpointDegrees);
      SmartDashboard.putNumber("Set Turret Pitch Position Setpoint", m_turretPitchSetpointDegrees);
   }
}