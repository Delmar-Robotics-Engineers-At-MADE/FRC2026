package frc.robot.subsystems;

// import com.playingwithfusion.TimeOfFlight;
// import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsystem extends SubsystemBase{

  static final int CANIDMotor = 30;
  static final double PositionTolerance = 10; // degrees
  static final double VelocityV = 20;  // RPM
  static final double OpenLoopSpeed = 0.3;
  static final double MRTOOR = 1/53.1429; // Motor Rotations To One Output Rotation ; main swerve is 5.49

  private SparkMax m_motor;
  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder m_encoder;

  private double m_debug_v_set = 0.0;

  // shuffleboard stuff
  private ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
  private ShuffleboardTab debugTab = Shuffleboard.getTab("Motor Debug");

  public IntakeSubsystem() {

    m_motor = new SparkMax(CANIDMotor, MotorType.kBrushless);
    closedLoopController = m_motor.getClosedLoopController();
    m_encoder = m_motor.getEncoder();

    motorConfig = new SparkMaxConfig();

    motorConfig.encoder
        .positionConversionFactor(MRTOOR)
        .velocityConversionFactor(MRTOOR);

    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed
        // loop slot, as it will default to slot 0.
        .p(0.4)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(0.2 * MRTOOR, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
        .feedForward
          // kV is now in Volts, so we multiply by the nominal voltage (12V)
          .kV(12.0 / (MRTOOR * 10700), ClosedLoopSlot.kSlot1); // 5767 is max RPM for Neo, 11000 for Neo 550

    motorConfig.closedLoop.maxMotion
        // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .cruiseVelocity(1000)
        .maxAcceleration(1000)
        .allowedProfileError(1)
        // Set MAXMotion parameters for velocity control in slot 1
        .maxAcceleration(500, ClosedLoopSlot.kSlot1)
        .cruiseVelocity(6000, ClosedLoopSlot.kSlot1)
        .allowedProfileError(1, ClosedLoopSlot.kSlot1);

    motorConfig.idleMode(IdleMode.kCoast);

    m_motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Dashboard indicators
    debugTab.addDouble("Intake Velocity (deg/min)", () -> getVelocity());
    debugTab.addDouble("Intake Velocity Setpoint (deg/min)", () -> getDebugV());

    setDefaultCommand(stopCommand());
  
  }

  public void moveVelocityControl (boolean in, double multiplier) {
    m_debug_v_set = multiplier * VelocityV * (in?1:-1);
    closedLoopController.setSetpoint(m_debug_v_set, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot1);
    // m_motor.set(OpenLoopSpeed * multiplier * (in?-1:1));
  }
  public void stop () {
    // closedLoopController.setReference(0, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot1);
    m_motor.set(0);
  }

  public Command moveVelocityOnceCmd(boolean in) {
    System.out.println("intake moving once " + in);
    return new InstantCommand(() -> moveVelocityControl(in, 1), this);
  }
  public Command moveVelocityCmd(boolean in) {
    System.out.println("intake moving " + in);
    return new RunCommand(() -> moveVelocityControl(in, 1), this);
  }
  public Command stopCommand() {
    return new InstantCommand(() -> stop(), this);
  }

  public double getAngle () {return m_encoder.getPosition();}
  public double getVelocity () {return m_encoder.getVelocity();}
  public double getDebugV () {return m_debug_v_set;}

}
