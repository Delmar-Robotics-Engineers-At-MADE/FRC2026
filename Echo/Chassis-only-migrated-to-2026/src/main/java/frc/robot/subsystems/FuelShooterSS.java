package frc.robot.subsystems;

// import com.playingwithfusion.TimeOfFlight;
// import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FuelShooterSS extends SubsystemBase{

  static final int CANIDShooterPort = 7;
  static final int CANIDShooterStar = 15;
  // static final int CANIDFusion = 1;  fusion line of flight sensor
  static final int DIONumPhotoEye = 6;
  static final double PositionTolerance = 10; // degrees
  static final double VelocityTolerance = 10000; // degrees per minute
  static final double VelocityV = 200000;  // degrees per minute
  static final double MRTOORTD = 360; // Motor Rotations To One Output Rotation To Degrees; main swerve is 5.49

  private SparkMax m_motorPort, m_motorStar;
  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder m_encoder;
  // private final TimeOfFlight m_tofSensor;

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
    motorConfig.encoder
        .positionConversionFactor(MRTOORTD)
        .velocityConversionFactor(MRTOORTD);

    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed
        // loop slot, as it will default to slot 0.
        .p(0.4 / MRTOORTD)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(0.0001 / MRTOORTD, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
        .feedForward.kV(nominalVoltage / (5767*MRTOORTD), ClosedLoopSlot.kSlot1); // Configure velocity gain on the feed forward closed feedback loop

    motorConfig.closedLoop.maxMotion
        // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .cruiseVelocity(1000*MRTOORTD)
        .maxAcceleration(1000*MRTOORTD)
        .allowedProfileError(PositionTolerance) // in degrees
        // Set MAXMotion parameters for velocity control in slot 1
        .maxAcceleration(500*MRTOORTD, ClosedLoopSlot.kSlot1)
        .cruiseVelocity(6000*MRTOORTD, ClosedLoopSlot.kSlot1)
        .allowedProfileError(VelocityTolerance, ClosedLoopSlot.kSlot1); // degrees per sec

    motorConfig.idleMode(IdleMode.kCoast);

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
  }

  public void moveVelocityControl (boolean in, double multiplier) {
    System.out.println(multiplier);
    closedLoopController.setSetpoint(multiplier * VelocityV * (in?1:-1), ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot1);
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
    return (Math.abs(m_encoder.getVelocity() - VelocityV) < VelocityTolerance) ;  
  }

  public double getVelocity () {return m_encoder.getVelocity();}

}
