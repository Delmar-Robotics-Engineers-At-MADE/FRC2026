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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsystem extends SubsystemBase{

  static final int CANIDMotor = 25;
  static final double TargetVelocityRPM = 1500;  // rotations per minute
  static final double OpenLoopSpeed = 0.3;
  static final double MRTOORTD = 360 / 5; // 5:1 reduction; Motor Rotations To One Output Rotation To Degrees; main swerve is 5.49

  private SparkMax m_motor;
  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder m_encoder;

  // shuffleboard stuff
  private ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
  private ShuffleboardTab debugTab = Shuffleboard.getTab("Motor Debug");

  public IntakeSubsystem() {

    m_motor = new SparkMax(CANIDMotor, MotorType.kBrushless);
    closedLoopController = m_motor.getClosedLoopController();
    m_encoder = m_motor.getEncoder();

    motorConfig = new SparkMaxConfig();

    motorConfig
      .inverted(true)
      .idleMode(IdleMode.kBrake)
      .openLoopRampRate(0.5)
      .smartCurrentLimit(40);

    m_motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Dashboard indicators
    debugTab.addDouble("Intake Velocity (rpm)", () -> getVelocity());

    setDefaultCommand(stopCommand());
  }

  public void moveVelocityControl (boolean in, double multiplier) {
    closedLoopController.setSetpoint(multiplier * TargetVelocityRPM * (in?-1:1), ControlType.kMAXMotionVelocityControl);
  }

  public void stop () {
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

  @Override
  public void periodic() {

    // Display subsystem values
    SmartDashboard.putNumber("Intake | Intake | Velocity Setpoint", closedLoopController.getSetpoint());
    SmartDashboard.putNumber("Intake | Intake | Applied Output", m_motor.getAppliedOutput());
  }

}
