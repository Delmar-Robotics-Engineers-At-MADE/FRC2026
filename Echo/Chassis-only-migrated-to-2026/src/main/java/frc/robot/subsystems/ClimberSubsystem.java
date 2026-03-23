package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
   boolean m_areClimbersHomed = false;
   boolean m_isClimberArmFaulted = false;

   // Most recently commanded climber setpoint
   double m_climberPositionSetpoint = ClimberSetpoints.kClimberStowedSetpoint;

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
      return Math.abs(m_leftClimberMotor.getPosition().getValueAsDouble() - desiredPosition) < ClimberSetpoints.kClimberPositionTolerance;
   }
   /**
    * Trigger: Is the left climber arm at the desired position?
    */
   public final Trigger isLeftArmAtPosition = new Trigger(
         () -> isLeftArmAt(this.m_climberPositionSetpoint));

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
    * Used to declare that the climber motors have been homed and their absolute position
    * can be set according to hard physical limits
    * This function is intended to be run at the beginning of autonomous init in
    * order to get the climber motors' output rotation relative to the local reference frame
    */
   private void setClimberArmsHomed() {
      m_areClimbersHomed = true;
      m_leftClimberMotor.setPosition(ClimberSetpoints.kClimberMotorHomingSetpoint);
      m_rightClimberMotor.setPosition(ClimberSetpoints.kClimberMotorHomingSetpoint);
   }

   /**
    * Used to return whether or not the climber motors have been homed and are
    * ready to be commanded to a particular position
    * This function must always be return true before utilizing closed-loop
    * position control on the yaw motor
    * 
    * @return Whether the climber arms have been homed
    */
   private boolean areClimbersArmsHomed() {
      return m_areClimbersHomed;
   }

   // TODO: Remove later. Temporary usage for testing climber motor functionality
   private void testSetClimberHomed() {
      setClimberArmsHomed();
   }

   /**
    * Command to home the climber arms to prep them for closed loop control
    */
   public Command homeClimberArms() {
      return run(() -> this.setClimberArmsHomed())
            .withName("Home Left Climber Arm");
   }

   //  TODO: Remove this later. Expose for now so we can manually set the left climber to "homed"
   public Command testCommandSetClimberHomed() {
      return this.runOnce(() -> this.testSetClimberHomed());
   }

       /**
     * Commands both arms to the given position with sync compensation.
     * Called continuously from a command's execute() loop.
     */
    // TODO: Reinstate the rest of this function once testing is complete on a single arm
    private void driveToPosition(double position) {
         if (!m_isClimberArmFaulted && m_areClimbersHomed) {
            this.m_climberPositionSetpoint = position;

            double leftPos = getLeftPosition();
            double rightPos = getRightPosition();
            double syncError = 0; //leftPos - rightPos; // positive = left is ahead
   
            // Feedforward sync correction — nudge voltage to keep arms together
            double syncFF = ClimberSubsystemConstants.kClimberArmSyncGain * syncError;
   
            m_leftClimberMotor.setControl(
               leftMMRequest.withPosition(position).withFeedForward(-syncFF)
            );
            // m_rightClimberMotor.setControl(
            //    rightMMRequest.withPosition(position).withFeedForward(syncFF)
            // );
        }
    }

    private void stop() {
        m_leftClimberMotor.setControl(neutralRequest);
        m_rightClimberMotor.setControl(neutralRequest);
    }

    public double getLeftPosition() {
        return m_leftClimberMotor.getPosition().getValueAsDouble();
    }

    public double getRightPosition() {
        return m_rightClimberMotor.getPosition().getValueAsDouble();
    }

    public boolean atSetpoint() {
        return Math.abs(getLeftPosition() - m_climberPositionSetpoint) < ClimberSetpoints.kClimberPositionTolerance
            && Math.abs(getRightPosition() - m_climberPositionSetpoint) < ClimberSetpoints.kClimberPositionTolerance;
    }

    public boolean isHomed() {
        return m_areClimbersHomed;
    }

    public boolean isSyncFaulted() {
        return m_isClimberArmFaulted;
    }

   /** Extend both arms up to grab the chain/bar. */
   public Command extendCommand() {
      return Commands.run(() -> driveToPosition(ClimberSetpoints.kClimberExtendedLevelZeroSetpoint), this)
         .until(this::atSetpoint)
         .finallyDo(interrupted -> { if (interrupted) stop(); })
         .withName("Climber Extend");
   }

   /** Retract both arms to pull the robot up. */
   public Command retractCommand() {
      return Commands.run(() -> driveToPosition(ClimberSetpoints.kClimberRetractedSetpoint), this)
         .until(this::atSetpoint)
         .finallyDo(interrupted -> { if (interrupted) stop(); })
         .withName("Climber Retract");
   }

   /** Full climb sequence: extend → wait for driver confirm → retract. */
   public Command climbSequenceCommand(java.util.function.BooleanSupplier confirmRetract) {
      return Commands.sequence(
         extendCommand(),
         Commands.waitUntil(confirmRetract),
         retractCommand()
      ).withName("Climb Sequence");
   }

   @Override
   public void periodic() {
      double leftPos = getLeftPosition();
      double rightPos = getRightPosition();
      double syncError = leftPos - rightPos;

      // Check sync fault
      if (m_areClimbersHomed && Math.abs(syncError) > ClimberSubsystemConstants.kCliberArmMaxDelta) {
         m_isClimberArmFaulted = true;
         m_leftClimberMotor.setControl(neutralRequest);
         m_rightClimberMotor.setControl(neutralRequest);
      }

      // Telemetry — publish sync error in degrees for easier human reading
      SmartDashboard.putNumber("Climber/LeftPosition", leftPos);
      SmartDashboard.putNumber("Climber/RightPosition", rightPos);
      SmartDashboard.putNumber("Climber/SyncErrorDeg", syncError * 360.0);
      SmartDashboard.putBoolean("Climber/IsHomed", m_areClimbersHomed);
      SmartDashboard.putBoolean("Climber/SyncFault", m_isClimberArmFaulted);
      SmartDashboard.putNumber("Climber/TargetPosition", m_climberPositionSetpoint);

      m_climberPositionSetpoint = SmartDashboard.getNumber("Set Climber Position (deg)", m_climberPositionSetpoint);
    }
}