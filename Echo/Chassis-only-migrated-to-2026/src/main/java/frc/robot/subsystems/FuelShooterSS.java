package frc.robot.subsystems;

// import com.playingwithfusion.TimeOfFlight;
// import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NeoMotorConstants;

public class FuelShooterSS extends SubsystemBase{

  static final int CANIDShooterPort = 7;
  static final int CANIDShooterStar = 15;
  // static final int CANIDFusion = 1;  fusion line of flight sensor
  static final int DIONumPhotoEye = 6;
  static final double PositionToleranceRotations = 0.1; // rotations
  static final double VelocityToleranceRPM = 1; // rotations per minute
  static final double TargetVeloctyRPM = 5000;  // rotations per minute

  private SparkMax m_motorPort, m_motorStar;
  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder m_encoder;

  // sim entities
  DCMotor m_maxSimGearbox = DCMotor.getNEO(2);
  private SparkMaxSim m_motorsSim;
  private FlywheelSim m_flywheelSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(m_maxSimGearbox, 0.0008246585, 1), m_maxSimGearbox);

  // shuffleboard stuff
  private ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
  private ShuffleboardTab debugTab = Shuffleboard.getTab("Motor Debug");

  public FuelShooterSS() {

    double nominalVoltage = 12.0;

    m_motorPort = new SparkMax(CANIDShooterPort, MotorType.kBrushless);
    m_motorStar = new SparkMax(CANIDShooterStar, MotorType.kBrushless);

    closedLoopController = m_motorPort.getClosedLoopController();
    m_encoder = m_motorPort.getEncoder();

    motorConfig = new SparkMaxConfig();

    motorConfig
      .idleMode(IdleMode.kCoast)
      .closedLoopRampRate(1.0) // seconds
      .openLoopRampRate(1.0) // seconds
      .smartCurrentLimit(60); // A

    motorConfig
      .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed
        // loop slot, as it will default to slot 0.
        .p(0.0007)
        .outputRange(-1, 1);

    motorConfig
      .closedLoop
        .maxMotion
          // Set MAXMotion parameters for MAXMotion Velocity control
          .maxAcceleration(10000)
          .cruiseVelocity(5000)
          .allowedProfileError(VelocityToleranceRPM); // degrees per sec

    // Configure velocity gain on the feed forward closed feedback loop
    // kV is in V/rpm so divide the nominal voltage by the NEO's max velocity in RPM
    motorConfig
      .closedLoop
        .feedForward.kV(nominalVoltage / NeoMotorConstants.kFreeSpeedRpm);

    m_motorPort.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // second motor inverted and following first
    motorConfig.follow(CANIDShooterPort, true);
    m_motorStar.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    
    // Dashboard indicators
    matchTab.addBoolean("Shooter Ready", () -> getVelocityReady())
        .withPosition(1, 6);
    debugTab.addDouble("Shooter Velocity (deg/min)", () -> getVelocity());

    // default command should be idle (no power, coasting)
    setDefaultCommand(coastCmd());

    // Zero the flywheel encoder on initialization
    m_encoder.setPosition(0.0);

    // set up sim entities
    m_motorsSim = new SparkMaxSim(m_motorPort, m_maxSimGearbox);
  }

  public void moveVelocityControl (boolean in, double multiplier) {
    System.out.println(multiplier);
    closedLoopController.setSetpoint(multiplier * TargetVeloctyRPM * (in?1:-1), ControlType.kMAXMotionVelocityControl);
  }

  public void moveOpenLoop (double speed) {
    m_motorPort.set(speed);
    m_motorStar.set(speed);
  }

  public Command moveVelocityCmd(boolean in) {
    return new RunCommand(() -> moveVelocityControl(in, 1), this);
  }
  
  public Command coastCmd() {
    return new RunCommand(() -> moveOpenLoop(0), this); // open loop 0 power, should coast
  }

  public boolean getVelocityReady () {
    return (Math.abs(m_encoder.getVelocity() - TargetVeloctyRPM) < VelocityToleranceRPM) ;  
  }

  public double getVelocity () {return m_encoder.getVelocity();}

  @Override
  public void periodic() {
    // Display subsystem values
    SmartDashboard.putNumber("Shooter | Flywheel | Applied Output", m_motorPort.getAppliedOutput());
    SmartDashboard.putNumber("Shooter | Flywheel | Current", m_motorPort.getOutputCurrent());
    SmartDashboard.putNumber("Shooter | Flywheel Follower | Applied Output", m_motorStar.getAppliedOutput());
    SmartDashboard.putNumber("Shooter | Flywheel Follower | Current", m_motorStar.getOutputCurrent());

    SmartDashboard.putNumber("Shooter | Flywheel | Target Velocity", TargetVeloctyRPM);
    SmartDashboard.putNumber("Shooter | Flywheel | Actual Velocity", m_encoder.getVelocity());
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
