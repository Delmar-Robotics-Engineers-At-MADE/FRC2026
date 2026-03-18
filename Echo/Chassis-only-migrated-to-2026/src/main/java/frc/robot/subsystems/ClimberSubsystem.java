package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Configs;
import frc.robot.Constants.ClimberSubsystemConstants;
import frc.robot.Constants.Falcon500MotorConstants;
import frc.robot.Constants.ClimberSubsystemConstants.ClimberSetpoints;
import frc.robot.Constants.ClimberSubsystemConstants.ClimberUnits;

public class ClimberSubsystem extends SubsystemBase{

   // Climber components
   TalonFX m_leftClimberMotor = new TalonFX(ClimberSubsystemConstants.kClimberLeftMotorCanId);
   TalonFX m_rightClimberMotor = new TalonFX(ClimberSubsystemConstants.kClimberRightMotorCanId);

   // Re-usable control requests
   private final MotionMagicVoltage leftMMRequest = new MotionMagicVoltage(0);
   private final MotionMagicVoltage rightMMRequest = new MotionMagicVoltage(0);
   private final NeutralOut neutralRequest = new NeutralOut();

   // Member state variables
   boolean m_isLeftClimberArmHomed = false;
   boolean m_isRightClimberArmHomed = false;

   // Most recently commanded climber setpoint
   double m_climberPositionSetpoint = 0.0;

   ClimberSubsystem() {

      TalonFXConfiguration leftConfig = Configs.ClimberSubsystem.leftConfig;
      TalonFXConfiguration rightConfig = Configs.ClimberSubsystem.rightConfig;
      
      m_leftClimberMotor.getConfigurator().apply(leftConfig);
      m_rightClimberMotor.getConfigurator().apply(rightConfig);

      SmartDashboard.putNumber("Set Climber Position (deg)", m_climberPositionSetpoint);
   }

   /**
    * Helper function to determining if the climber's position is within a small margin of error from the desired position
    * @param desiredPosition The position to compare against the actual position
    * @return Whether the current position of the left climber arm is "close enough" to the desired position
    */
   private boolean isLeftArmAt(double desiredPosition) {
      return Math.abs(m_leftClimberMotor.getPosition().getValueAsDouble() - desiredPosition) < ClimberSetpoints.kLeftClimberPositionTolerance;
   }
   /**
    * Trigger: Is the left climber arm at the desired position?
    */
   public final Trigger isLeftArmAtPosition = new Trigger(
         () -> isLeftArmAt(this.m_climberPositionSetpoint));

   /**
    * Used to return whether or not the left climber motor has been homed and is
    * ready to be commanded to a particular position
    * This function must always be return true before utilizing closed-loop
    * position control on the yaw motor
    * 
    * @return Whether the left climber arm has been homed
    */
   private boolean isLeftClimberArmHomed() {
      return m_isLeftClimberArmHomed;
   }

   /**
    * Gets the current state of the sensor being used to determine if the left climber
    * climber motor is at its homed position
    * This can include any failsafes that could be implented to protect against a
    * failing sensor
    * 
    * @return A boolean indicating if the the yaw motor has reached its homing
    *         setpoint
    */
   private BooleanSupplier getLeftArmAtHome() {
      return () ->((m_leftClimberMotor.getStatorCurrent().getValueAsDouble() > Falcon500MotorConstants.kMaxAllowedCurrent / 3) && 
                   Math.abs(m_leftClimberMotor.getVelocity().getValueAsDouble()) < ClimberUnits.kClimberNotMovingSafeThresholdDegreesPerSec);
   }

   /**
    * Used to declare that the left climber motor has been homed and its absolute position
    * can be set according to hard physical limits
    * This function is intended to be run at the beginning of autonomous init in
    * order to get the left climber motor's output rotation relative to the local reference frame
    */
   private void setLeftClimberHomed() {
      m_isLeftClimberArmHomed = true;
      m_leftClimberMotor.setPosition(ClimberSetpoints.kClimberMotorHomingSetpoint);
   }
   
   /**
    * Used to declare that the right climber motor has been homed and its absolute position
    * can be set according to hard physical limits
    * This function is intended to be run at the beginning of autonomous init in
    * order to get the right climber motor's output rotation relative to the local reference frame
    */
   private void setRightClimberHomed() {
      m_isLeftClimberArmHomed = true;
      m_leftClimberMotor.setPosition(ClimberSetpoints.kClimberMotorHomingSetpoint);
   }

   /**
    * Tells both motors to explicitly stop moving
    */
   private void stop() {
      m_leftClimberMotor.setControl(neutralRequest);
      m_rightClimberMotor.setControl(neutralRequest);
   }

   // TODO: Remove later. Temporary usage for testing climber motor functionality
   private void testSetClimberHomed() {
      setLeftClimberHomed();
      setRightClimberHomed();
   }

   /**
    * Command to home the turret yaw motor at a low duty cycle and set its starting
    * parameters to allow closed loop position control
    */
   public Command homeLeftClimberArm() {
      return run(() -> this.moveLeftArm(0.1))
            .until(getLeftArmAtHome())
            .andThen(() -> this.m_leftClimberMotor.stopMotor())
            .andThen(() -> this.setLeftClimberHomed())
            .withName("Home Left Climber Arm");
   }

   //  TODO: Remove this later. Expose for now so we can manually set the left climber to "homed"
   public Command testCommandSetClimberHomed() {
      return this.runOnce(() -> this.testSetClimberHomed());
   }

   private void moveLeftArm(double position) {
      m_leftClimberMotor.set(position);
   }

   @Override
   public void periodic() {
      m_climberPositionSetpoint = SmartDashboard.getNumber("Set Climber Position (deg)", m_climberPositionSetpoint);
   }
}