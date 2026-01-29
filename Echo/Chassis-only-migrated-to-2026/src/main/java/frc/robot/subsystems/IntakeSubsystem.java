package frc.robot.subsystems;

// import com.playingwithfusion.TimeOfFlight;
// import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsystem extends SubsystemBase{

  static final int CANIDMotor = 44;
  static final double PositionTolerance = 10; // degrees
  static final double VelocityV = 25000;  // degrees per minute
  static final double OpenLoopSpeed = 0.3;
  static final double MRTOORTD = 360 / 20; // Motor Rotations To One Output Rotation To Degrees; main swerve is 5.49

  private SparkMax m_motor;
  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder m_encoder;

  // shuffleboard stuff
  private ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
  private ShuffleboardTab debugTab = Shuffleboard.getTab("Motor Debug");

  public IntakeSubsystem() {

    double nominalVoltage = 12.0;

    m_motor = new SparkMax(CANIDMotor, MotorType.kBrushless);
    closedLoopController = m_motor.getClosedLoopController();
    m_encoder = m_motor.getEncoder();

    motorConfig = new SparkMaxConfig();
    motorConfig.encoder
        .positionConversionFactor(MRTOORTD)
        .velocityConversionFactor(MRTOORTD);

    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed
        // loop slot, as it will default to slot 0.
        .p(0.4 /MRTOORTD)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(0.001/MRTOORTD, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
        .feedForward.kV(nominalVoltage / (5767*MRTOORTD), ClosedLoopSlot.kSlot1); // Specifically configure feedforward velocity gain (now as a factor of voltage)


    motorConfig.closedLoop.maxMotion
        // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .cruiseVelocity(1000*MRTOORTD)
        .maxAcceleration(1000*MRTOORTD)
        .allowedProfileError(PositionTolerance) // in degrees
        // Set MAXMotion parameters for velocity control in slot 1
        .maxAcceleration(1000*MRTOORTD, ClosedLoopSlot.kSlot1)
        .cruiseVelocity(60000*MRTOORTD, ClosedLoopSlot.kSlot1)
        .allowedProfileError(MRTOORTD, ClosedLoopSlot.kSlot1); // degrees per sec

    motorConfig.idleMode(IdleMode.kBrake);

    m_motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Dashboard indicators
    debugTab.addDouble("Intake Velocity (deg/min)", () -> getVelocity());

    setDefaultCommand(stopCommand());
  }

  public void moveVelocityControl (boolean in, double multiplier) {
    closedLoopController.setSetpoint(multiplier * VelocityV * (in?-1:1), ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot1);
    // m_motor.set(OpenLoopSpeed * (in?-1:1));
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

}
