package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
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

public class FuelShooterSS extends SubsystemBase{

  // Flywheel components
  private SparkMax m_motorPort, m_motorStar, m_feederMotor;
  private SparkClosedLoopController m_flywheelClosedLoopController;
  private RelativeEncoder m_flywheelEncoder;

  // Turret components
  private SparkMax m_turretYawMotor, m_turretPitchMotor; // rotation and hood control motors
  private SparkClosedLoopController m_turretYawClosedLoopController, m_turretPitchClosedLoopController;
  private RelativeEncoder m_turretYawEncoder, m_turretPitchEncoder;
  // ########### SIM ###########

  DCMotor m_maxSimGearbox = DCMotor.getNEO(2);
  private SparkMaxSim m_motorsSim;
  private FlywheelSim m_flywheelSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(m_maxSimGearbox, 0.00316577577, 1), m_maxSimGearbox);
  
  // ###########################

  private final SendableChooser<Double> m_flywheelVelocityChooser = new SendableChooser<Double>();

  // Member variables for subsystem state management
  private double m_flywheelTargetVelocity = ShooterSubsystemConstants.FlywheelSetpoints.kShootRpm;

  public FuelShooterSS() {

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

    // Zero flywheel encoder on initialization
    m_flywheelEncoder.setPosition(0.0);

    // TODO: Add code to zero turret yaw and pitch at some point early on (not sure when we are allowed to move motors)

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
      () -> isFlywheelAt(ShooterSubsystemConstants.FlywheelSetpoints.kShootRpm) || 
            m_flywheelEncoder.getVelocity() > ShooterSubsystemConstants.FlywheelSetpoints.kShootRpm
  );

  public final Trigger isFlywheelSpinningBackwards = new Trigger(
      () -> isFlywheelAt(-ShooterSubsystemConstants.FlywheelSetpoints.kShootRpm) || 
            m_flywheelEncoder.getVelocity() < -ShooterSubsystemConstants.FlywheelSetpoints.kShootRpm
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
    m_turretYawMotor.set(dutyCycle);
  }

    /**
   * Used for testing and backup ability for manual control of the turret's hood position
   * @param dutyCycle A value from [-1, 1]
   */
  private void moveTurretPitch(double dutyCycle) {
    m_turretPitchMotor.set(dutyCycle);
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
  * applied dury cycle. Once the command ends, the motors will stop.
  */
  public Command moveTurretRotationManual(double dutyCycle) {
    return this.startEnd(
        () -> {
          this.moveTurretYaw(dutyCycle);
        }, () -> {
          this.m_turretYawMotor.stopMotor();
        }).withName("Turning turret");
  }

  /**
  * Command to manually control the turret's hood. While being commanded, the turret hood will move with the
  * applied dury cycle. Once the command ends, the motors will stop.
  */
  public Command moveTurretHoodManual(double dutyCycle) {
    return this.startEnd(
        () -> {
          this.moveTurretPitch(dutyCycle);
        }, () -> {
          this.m_turretPitchMotor.stopMotor();
        }).withName("Moving turret hood");
  }

  public double getVelocity () {return m_flywheelEncoder.getVelocity();}

  @Override
  public void periodic() {
    // Display subsystem values
    SmartDashboard.putNumber("Shooter | Flywheel | Applied Output", m_motorPort.getAppliedOutput());
    SmartDashboard.putNumber("Shooter | Flywheel | Current", m_motorPort.getOutputCurrent());
    SmartDashboard.putNumber("Shooter | Flywheel | Velocity Setpoint", m_flywheelClosedLoopController.getMAXMotionSetpointVelocity());

    SmartDashboard.putNumber("Shooter | Flywheel Follower | Applied Output", m_motorStar.getAppliedOutput());
    SmartDashboard.putNumber("Shooter | Flywheel Follower | Current", m_motorStar.getOutputCurrent());

    SmartDashboard.putNumber("Shooter | Flywheel | Target Velocity", m_flywheelTargetVelocity);
    SmartDashboard.putNumber("Shooter | Flywheel | Actual Velocity", m_flywheelEncoder.getVelocity());

    SmartDashboard.putBoolean("Is Flywheel Spinning", isFlywheelSpinning.getAsBoolean());
    SmartDashboard.putBoolean("Is Flywheel Stopped", isFlywheelStopped.getAsBoolean());
  
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
