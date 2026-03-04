package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.FloatArrayPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.ShooterSubsystemConstants;
import frc.robot.Constants.ShooterSubsystemConstants.FeederSetpoints;
import frc.robot.Constants.ShooterSubsystemConstants.FlywheelSetpoints;
import frc.robot.Constants.ShooterSubsystemConstants.TurretSetpoints;

public class FuelShooterSubsystem extends SubsystemBase{

  // Flywheel components
  private SparkMax m_motorPort, m_motorStar, m_feederMotor;
  private SparkClosedLoopController m_flywheelClosedLoopController;
  private RelativeEncoder m_flywheelEncoder;

  // Turret components
  private SparkMax m_turretYawMotor, m_turretPitchMotor; // rotation and hood control motors
  private SparkClosedLoopController m_turretYawClosedLoopController, m_turretPitchClosedLoopController;
  private RelativeEncoder m_turretYawEncoder, m_turretPitchEncoder;

  // Sensors
  private final DigitalInput m_hallEffectYaw = new DigitalInput(0);
  private final DigitalInput m_hallEffectPitch = new DigitalInput(1);

  // ########### SIM ###########

  DCMotor m_maxSimGearbox = DCMotor.getNEO(2);
  private SparkMaxSim m_motorsSim;
  private FlywheelSim m_flywheelSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(m_maxSimGearbox, 0.00316577577, 1), m_maxSimGearbox);
  
  // ###########################

  private final SendableChooser<Double> m_flywheelVelocityChooser = new SendableChooser<Double>();

  // Member variables for subsystem state management
  private double m_flywheelTargetVelocity = ShooterSubsystemConstants.FlywheelSetpoints.kShootRpm;

  double m_turretYawSetpointDegrees = 0.0;
  boolean m_isTurretYawHomed = false;
  double m_turretPitchSetpointDegrees = 0.0;
  boolean m_isTurretPitchHomed = false;

  public FuelShooterSubsystem() {

    // Initialize flywheel motors
    m_motorPort = new SparkMax(ShooterSubsystemConstants.kFlywheelMotorCanId, MotorType.kBrushless);
    m_motorStar = new SparkMax(ShooterSubsystemConstants.kFlywheelFollowerMotorCanId, MotorType.kBrushless);
    m_feederMotor = new SparkMax(ShooterSubsystemConstants.kFeederMotorCanId, MotorType.kBrushless);

    // Flywheel motors are connected together so use the leader's closed loop controller and encoder for control
    m_flywheelClosedLoopController = m_motorPort.getClosedLoopController();
    m_flywheelEncoder = m_motorPort.getEncoder();

    // Initialize shooter pointing motors (yaw motor controls the shooter's direction while the pitch motor controls hood position)
    m_turretYawMotor = new SparkMax(ShooterSubsystemConstants.kTurretYawMotorCanId, MotorType.kBrushless);
    m_turretPitchMotor = new SparkMax(ShooterSubsystemConstants.kTurretPitchMotorCanId, MotorType.kBrushless);

    // Initialize individual closed loop controllers and motors for each of the turret components individually
    m_turretYawClosedLoopController = m_turretYawMotor.getClosedLoopController();
    m_turretYawEncoder = m_turretYawMotor.getEncoder();
    m_turretPitchClosedLoopController = m_turretPitchMotor.getClosedLoopController();
    m_turretPitchEncoder = m_turretPitchMotor.getEncoder();

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
    m_feederMotor.configure(
        Configs.ShooterSubsystem.feederConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_turretYawMotor.configure(
        Configs.ShooterSubsystem.turretYawConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_turretPitchMotor.configure(
        Configs.ShooterSubsystem.turretPitchConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Zero encoders on initialization
    m_flywheelEncoder.setPosition(0.0);
    m_turretYawEncoder.setPosition(0.0);
    m_turretPitchEncoder.setPosition(0.0);

    // set up sim entities
    m_motorsSim = new SparkMaxSim(m_motorPort, m_maxSimGearbox);

    m_flywheelVelocityChooser.setDefaultOption("Low Speed (1500 rpm)", 1500.0);
    m_flywheelVelocityChooser.addOption("Mid Speed (2500 rpm)", 2500.0);
    m_flywheelVelocityChooser.addOption("Full Speed (4500 rpm)", FlywheelSetpoints.kShootRpm);
    SmartDashboard.putData("Velocity Target", m_flywheelVelocityChooser);
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
            m_flywheelEncoder.getVelocity() > this.m_flywheelTargetVelocity
  );

  public final Trigger isFlywheelSpinningBackwards = new Trigger(
      () -> isFlywheelAt(-this.m_flywheelTargetVelocity) || 
            m_flywheelEncoder.getVelocity() < -this.m_flywheelTargetVelocity
  );

  /** 
  * Trigger: Is the flywheel stopped?
  */
  public final Trigger isFlywheelStopped = new Trigger(() -> isFlywheelAt(0));

  /**
  * Drive the flywheels to their set velocity. This will use MAXMotion
  * velocity control which will allow for a smooth acceleration and deceleration to the mechanism's
  * setpoint.
  */
  private void setFlywheelVelocity(double velocity) {
    m_flywheelClosedLoopController.setSetpoint(velocity, ControlType.kMAXMotionVelocityControl);
  }

  /** Set the feeder motor power in the range of [-1, 1]. */
  private void setFeederPower(double power) {
    m_feederMotor.set(power);
  }

  /**
   * Used for testing and backup ability for manual control of the turret's rotation
   * @param dutyCycle A value from [-1, 1]
   */
  private void moveTurretYaw(double dutyCycle){

    // Clamp the applied duty cycle to 30% for safety in testing
    double actualAppliedDutyCycle = Math.max(-0.3, Math.min(0.3, dutyCycle));
    m_turretYawMotor.set(actualAppliedDutyCycle);
  }

    /**
  * Used to command the turret's yaw motor to a particular absolute position in degrees
  * @param position Absolute output position of the turret's rotation in degrees
  */
  private void moveTurretYawToPosition(double position) {
    if (isTurretYawHomed())
    {
      // Calculate the position movement that needs to be performed from the current absolute position
      // in order to reach the commanded output position
      double currentYawPosition = m_turretYawEncoder.getPosition();
    
      // TODO: Implement logic to move position in direction that avoids the turret pitch deadzone

      // TODO: Update this call to use the position that is being passed in after testing
      m_turretYawClosedLoopController.setSetpoint(m_turretYawSetpointDegrees, ControlType.kMAXMotionPositionControl);
    }
  }

  /**
   * Used to return whether or not the yaw turret motor has been homed and is ready to be commaned to a particular position
   * This function must always be return true before utilizing closed-loop position control on the yaw motor
   * @return Whether the yaw turret has been homed
   */
  private boolean isTurretYawHomed() {
    return m_isTurretYawHomed;
  }

  /**
  * Gets the current state of the sensor being used to determine if the yaw turret is at its homed position
  * This can include any failsafes that could be implented to protect against a failing sensor
  * @return A boolean indicating if the the yaw motor has reached its homing setpoint
  */
  public boolean getTurretYawAtHome() {
    return !m_hallEffectYaw.get();
  }

  /**
   * Used to declare that the yaw turret has been homed and its absolute position can be set according to hard physical limits
   * This function is intended to be run at the beginning of autonomous init in order to get the turret's absolute output rotation
   */
  private void setTurretYawHomed() {
    m_isTurretYawHomed = true;
    m_turretYawEncoder.setPosition(TurretSetpoints.kYawMotorHomingSetpoint);
  }

  /**
   * Used for testing and backup ability for manual control of the turret's hood position
   * @param dutyCycle A value from [-1, 1]
   */
  private void moveTurretPitch(double dutyCycle) {

    // Clamp the applied duty cycle to 60% for safety in testing
    double actualAppliedDutyCycle = Math.max(-0.3, Math.min(0.3, dutyCycle));
    m_turretPitchMotor.set(actualAppliedDutyCycle);
  }

  /**
  * Used to command the turret's hood to a particular absolute position in degrees
  * @param position Absolute output position of the turret's hood in degrees (launch angle)
  */
  private void moveTurretPitchToPosition(double position) {
    if (isTurretPitchHomed())
    {
      // Calculate the position movement that needs to be performed from the current absolute position
      // in order to reach the commanded output position
      double currentPitchPosition = m_turretPitchEncoder.getPosition();

      // TODO: Implement logic to move position in direction that avoids the turret pitch deadzone

      m_turretPitchClosedLoopController.setSetpoint(m_turretPitchSetpointDegrees, ControlType.kMAXMotionPositionControl);
    }
  }

  /**
   * Used to return whether or not the pitch turret motor has been homed and is ready to be commaned to a particular position
   * This function must always be return true before utilizing closed-loop position control on the pitch motor
   * @return Whether the pitch turret has been homed
   */
  private boolean isTurretPitchHomed() {
    return m_isTurretPitchHomed;
  }

  /**
  * Gets the current state of the sensor being used to determine if the pitch turret is at its homed position
  * This can include any failsafes that could be implented to protect against a failing sensor
  * @return A boolean indicating if the the pitch motor has reached its homing setpoint
  */
  public boolean getTurretPitchAtHome() {
    return !m_hallEffectPitch.get();
  }

  /**
   * Used to declare that the pitch turret has been homed and its absolute position can be set according to hard physical limits
   * This function is intended to be run at the beginning of autonomous init in order to get the hood's absolute output rotation
   */
  private void setTurretPitchHomed() {
    m_isTurretPitchHomed = true;
    m_turretPitchEncoder.setPosition(TurretSetpoints.kPitchMotorHomingSetpoint);
  }

    /**
   * Command to run the flywheel motors. When the command is interrupted, e.g. the button is released,
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

  /**
   * Command to run the feeder and flywheel motors. When the command is interrupted, e.g. the button is released,
   * the motors will stop.
   */
  public Command runFeederCommand() {
    return this.startEnd(
        () -> {
          this.setFeederPower(Constants.ShooterSubsystemConstants.FeederSetpoints.kFeed);
        }, () -> {
          this.setFeederPower(0.0);
        }).withName("Feeding");
  }

  /**
  * Meta-command to operate the shooter. The Flywheel starts spinning up and when it reaches
  * the desired speed it starts the Feeder.
  */
  public Command runShooterCommand() {
    return this.startEnd(
      () -> this.setFlywheelVelocity(m_flywheelTargetVelocity),
      () -> m_motorPort.stopMotor()
    ).until(isFlywheelSpinning).andThen(
      this.startEnd(
        () -> {
          this.setFlywheelVelocity(m_flywheelTargetVelocity);
          this.setFeederPower(FeederSetpoints.kFeed);
        }, () -> {
          m_motorPort.stopMotor();
          m_feederMotor.stopMotor();
        })
    ).withName("Shooting");
  }

  /**
  * Command to manually control the turret's rotation. While being commanded, the turret will move with the
  * applied duty cycle. Once the command ends, the motors will stop.
  */
  public Command moveTurretRotationManual(DoubleSupplier dutyCycle) {
    return this.startEnd(
        () -> {
          this.moveTurretYaw(dutyCycle.getAsDouble());
        }, () -> {
          this.m_turretYawMotor.stopMotor();
        }).withName("Turning turret");
  }

  /**
  * Command to home the turret yaw motor at a low duty cycle and set its starting 
  * parameters to allow closed loop position control
  */
  public Command homeTurretYaw() {
    return this.startEnd(
      () -> {
        this.moveTurretYaw(0.1);
      }, () -> {
        this.setTurretYawHomed();
      }).withName("Homing yaw turret motor");
  }

  /**
  * Commands the turret to rotate to an absolute position
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
  * Command to manually control the turret's hood. While being commanded, the turret hood will move with the
  * applied duty cycle. Once the command ends, the motors will stop.
  */
  public Command moveTurretHoodManual(DoubleSupplier dutyCycle) {
    return this.startEnd(
        () -> {
          this.moveTurretPitch(dutyCycle.getAsDouble());
        }, () -> {
          this.m_turretPitchMotor.stopMotor();
        }).withName("Moving turret hood");
  }

  /**
  * Command to home the turret yaw motor at a low duty cycle and set its starting 
  * parameters to allow closed loop position control
  */
  public Command homeTurretPitch() {
    return this.startEnd(
      () -> {
        this.moveTurretPitch(0.1);
      }, () -> {
        this.setTurretPitchHomed();
      }).withName("Homing yaw turret motor");
  }

  /**
  * Commands the turret hood to move to an absolute position
  * @param position The desired absolute position of the turret hood
  */
  public Command commandTurretPitchToPosition(double position) {
    return this.startEnd(
      () -> {
        this.moveTurretPitchToPosition(position);
      }, () -> {
        this.m_turretYawMotor.stopMotor();
      }).withName("Rotating turret yaw to position");
  }

  public double getVelocity () {return m_flywheelEncoder.getVelocity();}

  @Override
  public void periodic() {
    // Display subsystem values
    SmartDashboard.putNumber("Shooter | Flywheel | Applied Output", m_motorPort.getAppliedOutput());
    SmartDashboard.putNumber("Shooter | Flywheel | Current", m_motorPort.getOutputCurrent());
    SmartDashboard.putNumber("Shooter | Flywheel | Velocity Setpoint", m_flywheelClosedLoopController.getMAXMotionSetpointVelocity());
    SmartDashboard.putNumber("Shooter | Turret Yaw | Velocity Setpoint", m_turretYawClosedLoopController.getMAXMotionSetpointVelocity());
    SmartDashboard.putNumber("Shooter | Turret Yaw | Current", m_turretYawMotor.getOutputCurrent());
    SmartDashboard.putNumber("Shooter | Turret Pitch | Velocity Setpoint", m_turretPitchClosedLoopController.getMAXMotionSetpointVelocity());
    SmartDashboard.putNumber("Shooter | Turret Pitch | Current", m_turretPitchMotor.getOutputCurrent());

    // Temps
    SmartDashboard.putNumber("Shooter | Flywheel Leader | Temperature (deg C)", m_motorPort.getMotorTemperature());
    SmartDashboard.putNumber("Shooter | Flywheel Follower | Temperature (deg C)", m_motorStar.getMotorTemperature());
    SmartDashboard.putNumber("Shooter | Feeder | Temperature (deg C)", m_feederMotor.getMotorTemperature());
    SmartDashboard.putNumber("Shooter | Turret Yaw | Temperature (deg C)", m_turretYawMotor.getMotorTemperature());
    SmartDashboard.putNumber("Shooter | Turret Pitch | Temperature (deg C)", m_turretPitchMotor.getMotorTemperature());

    SmartDashboard.putNumber("Shooter | Flywheel Follower | Applied Output", m_motorStar.getAppliedOutput());
    SmartDashboard.putNumber("Shooter | Flywheel Follower | Current", m_motorStar.getOutputCurrent());

    SmartDashboard.putNumber("Shooter | Flywheel | Target Velocity", m_flywheelTargetVelocity);
    SmartDashboard.putNumber("Shooter | Flywheel | Actual Velocity", m_flywheelEncoder.getVelocity());

    SmartDashboard.putBoolean("Is Flywheel Spinning", isFlywheelSpinning.getAsBoolean());
    SmartDashboard.putBoolean("Is Flywheel Stopped", isFlywheelStopped.getAsBoolean());

    m_turretYawSetpointDegrees = SmartDashboard.getNumber("Set Turret Yaw Position", 0.0);
    m_turretPitchSetpointDegrees = SmartDashboard.getNumber("Set Turret Pitch Position", 0.0);
  
    // Sensors
    SmartDashboard.putBoolean("Hall Effect Sensor Detection", m_hallEffectYaw.get());

    m_flywheelTargetVelocity = m_flywheelVelocityChooser.getSelected();
  }

  public void simulationPeriodic() {
    m_flywheelSim.setInput(m_motorPort.getAppliedOutput() * RobotController.getInputVoltage());
    m_flywheelSim.update(0.02);

    // Now, we update the Spark Flex
    m_motorsSim.iterate(
      m_flywheelSim.getAngularVelocityRPM(),
      RobotController.getInputVoltage(), // Simulated battery voltage, in Volts
      0.02); // Time interval, in Seconds

    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(m_flywheelSim.getCurrentDrawAmps()));
  }
}
