package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
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

   private static double kSeatCycleCount = 10;

   private double m_actualRetractPosition = ClimberSetpoints.kClimberRetractedSetpoint;
   private boolean m_hasSeatedRetract = false;
   private int m_seatCounter = 0;

   // Climber components
   TalonFX m_leftClimberMotor = new TalonFX(ClimberSubsystemConstants.kClimberLeftMotorCanId);
   TalonFX m_rightClimberMotor = new TalonFX(ClimberSubsystemConstants.kClimberRightMotorCanId);

   // Re-usable control requests
   private final MotionMagicVoltage leftMMRequest = new MotionMagicVoltage(0);
   private final MotionMagicVoltage rightMMRequest = new MotionMagicVoltage(0);
   private final StaticBrake brakeRequest = new StaticBrake();
   private final NeutralOut neutralRequest = new NeutralOut();

   private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

   // Member state variables
   boolean m_areClimbersHomed = false;
   boolean m_isClimberArmFaulted = false;

   // Most recently commanded climber setpoint
   double m_climberPositionSetpoint = ClimberSetpoints.kClimberStowedSetpoint;

   // Tuning constants
   private double mt_climberkP = ClimberUnits.kClimberkP;
   private double mt_climberkI = ClimberUnits.kClimberkI;
   private double mt_climberkD = ClimberUnits.kClimberkD;
   private double mt_climberkG = ClimberUnits.kClimberkG;
   private double mt_climberkS = ClimberUnits.kClimberkS;
   private double mt_climberkV = ClimberUnits.kClimberkV;
   private double mt_climberSyncGain = ClimberSubsystemConstants.kClimberArmSyncGain;
   private double mt_climberMaxDelta = ClimberSubsystemConstants.kCliberArmMaxDelta;

   public ClimberSubsystem() {

      TalonFXConfiguration leftConfig = Configs.ClimberSubsystem.leftConfig;
      TalonFXConfiguration rightConfig = Configs.ClimberSubsystem.rightConfig;
      
      m_leftClimberMotor.getConfigurator().apply(leftConfig);
      m_rightClimberMotor.getConfigurator().apply(rightConfig);

      SmartDashboard.putNumber("Set Climber Position (deg)", m_climberPositionSetpoint);

      // TODO: Remove these later; tuning constants
      SmartDashboard.putNumber("Set Climber/kP", mt_climberkP);
      SmartDashboard.putNumber("Set Climber/kI", mt_climberkI);
      SmartDashboard.putNumber("Set Climber/kD", mt_climberkD);
      SmartDashboard.putNumber("Set Climber/kG", mt_climberkG);
      SmartDashboard.putNumber("Set Climber/kS", mt_climberkS);
      SmartDashboard.putNumber("Set Climber/kV", mt_climberkV);
      SmartDashboard.putNumber("Set Climber/Sync Gain", mt_climberSyncGain);
      SmartDashboard.putNumber("Set Climber/Max Delta", mt_climberMaxDelta);

      setClimberArmsHomed();
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
                   Math.abs(m_leftClimberMotor.getVelocity().getValueAsDouble()) < ClimberUnits.kClimberNotMovingSafeThreshold);
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
    * Command to home the climber arms to prep them for closed loop control
    */
   public Command homeClimberArms() {
      return run(() -> this.setClimberArmsHomed())
            .withName("Home Left Climber Arm");
   }

   public void updateRetractCurrentDetection() {
      if (m_leftClimberMotor.getStatorCurrent().getValueAsDouble() > 30.0 
       && m_rightClimberMotor.getStatorCurrent().getValueAsDouble() > 30.0) {
         m_seatCounter++;
      } else {
         m_seatCounter = 0;
      }

      if (m_seatCounter >= kSeatCycleCount) {
         m_actualRetractPosition = m_leftClimberMotor.getPosition().getValueAsDouble(); // capture actual position
         m_hasSeatedRetract = true;
      }
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

   private boolean hasSeatedRetract() {
      return m_hasSeatedRetract;
   }

    public void moveArmsDown() {
      m_leftClimberMotor.setControl(dutyCycleOut.withOutput(-0.25));
      m_rightClimberMotor.setControl(dutyCycleOut.withOutput(-0.25));
    }

    public void moveArmsUp() {
      m_leftClimberMotor.setControl(dutyCycleOut.withOutput(0.25));
      m_rightClimberMotor.setControl(dutyCycleOut.withOutput(0.25));
    }

    private void stop() {
      m_leftClimberMotor.setControl(neutralRequest);
      m_rightClimberMotor.setControl(neutralRequest);
    }

    private void hold()  {
      m_leftClimberMotor.setControl(brakeRequest);
      m_rightClimberMotor.setControl(brakeRequest);
    }

    public double getLeftPosition() {
        return m_leftClimberMotor.getPosition().getValueAsDouble();
    }

    public double getRightPosition() {
        return m_rightClimberMotor.getPosition().getValueAsDouble();
    }

    public BooleanSupplier atSetpoint() {
        return () -> Math.abs(getLeftPosition() - m_climberPositionSetpoint) < ClimberSetpoints.kClimberPositionTolerance
                  && Math.abs(getRightPosition() - m_climberPositionSetpoint) < ClimberSetpoints.kClimberPositionTolerance;
    }

    public boolean isHomed() {
        return m_areClimbersHomed;
    }

    public boolean isSyncFaulted() {
        return m_isClimberArmFaulted;
    }

   public Command homeArms() {
      return Commands.run(() -> homeClimberArms(), this)
         .withName("Setting arms as homed");
   }

   /** Extend both arms up to grab the chain/bar. */
   public Command extendCommand() {
      return Commands.run(() -> driveToPosition(ClimberSetpoints.kClimberExtendedLevelZeroSetpoint), this)
         .until(atSetpoint())
         .finallyDo(interrupted -> { if (interrupted) stop(); })
         .withName("Climber Extend");
   }

   /** Simple retract both arms to pull the robot up. */
   public Command simpleRetractCommand() {
      return Commands.run(() -> driveToPosition(ClimberSetpoints.kClimberRetractedSetpoint + 0.2), this)
         .until(atSetpoint())
         .finallyDo(interrupted -> { if (interrupted) stop(); })
         .withName("Climber Retract");
   }

   /**
    * Full-on retract command with curret detection and saving off of actual robot arm position
    */
   public Command retractCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> {
         m_hasSeatedRetract = false; 
         m_seatCounter = 0;
        }, this),
        Commands.run(() -> {
            driveToPosition(ClimberSetpoints.kClimberRetractedSetpoint);
            updateRetractCurrentDetection();
        }, this)
        .until(this::hasSeatedRetract),
        Commands.run(this::hold, this)
    ).finallyDo(interrupted -> { if (interrupted) stop(); })
     .withName("Climber Retract");
}

   /** Full climb sequence: extend → wait for driver confirm → retract. */
   public Command climbSequenceCommand(BooleanSupplier confirmRetract) {
      return Commands.sequence(
         extendCommand(),
         Commands.waitUntil(confirmRetract),
         simpleRetractCommand()
      ).withName("Climb Sequence");
   }

   /**
    * Simple command for moving robot arms up
    */
   public Command moveArmsUpCommand() {
      return Commands.startEnd(() -> {
            moveArmsUp();
         }, () ->  {
            m_leftClimberMotor.setControl(neutralRequest);
            m_rightClimberMotor.setControl(neutralRequest);
         }, this)
         .withName("Move arms up");
   }

   /**
    * Simple command for moving robot arms up
    */
   public Command moveArmsDownCommand() {
      return Commands.startEnd(() -> {
            moveArmsDown();
         }, () ->  {
            m_leftClimberMotor.setControl(neutralRequest);
            m_rightClimberMotor.setControl(neutralRequest);
         }, this)
         .withName("Move arms down");
   }

   // TODO: Remove this or move it to a shared place later. There is a matching function in the shooter class
   private boolean hasChanged(double a, double b) {
      return Math.abs(a - b) > 1e-6;
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

      double newClimberkP = SmartDashboard.getNumber("Set Climber/kP", mt_climberkP);
      double newClimberkI = SmartDashboard.getNumber("Set Climber/kI", mt_climberkI);
      double newClimberkD = SmartDashboard.getNumber("Set Climber/kD", mt_climberkD);
      double newClimberkG = SmartDashboard.getNumber("Set Climber/kG", mt_climberkG);
      double newClimberkS = SmartDashboard.getNumber("Set Climber/kS", mt_climberkS);
      double newClimberkV = SmartDashboard.getNumber("Set Climber/kV", mt_climberkV);

      if (hasChanged(newClimberkP, mt_climberkP) || hasChanged(newClimberkI, mt_climberkI) || hasChanged(newClimberkD, mt_climberkD) ||
          hasChanged(newClimberkG, mt_climberkG) || hasChanged(newClimberkS, mt_climberkS) || hasChanged(newClimberkV, mt_climberkV)) {
            
            Slot0Configs leftSlot0 = new Slot0Configs();
            m_leftClimberMotor.getConfigurator().refresh(leftSlot0);

            leftSlot0
               .withKP(newClimberkP)
               .withKI(newClimberkI)
               .withKD(newClimberkD)
               .withKG(newClimberkG)
               .withKS(newClimberkS)
               .withKV(newClimberkV);

            m_leftClimberMotor.getConfigurator().apply(leftSlot0);
            m_rightClimberMotor.getConfigurator().apply(leftSlot0);

            mt_climberkP = newClimberkP;
            mt_climberkI = newClimberkI;
            mt_climberkD = newClimberkD;
            mt_climberkG = newClimberkG;
            mt_climberkS = newClimberkS;
            mt_climberkV = newClimberkV;
          }

      m_climberPositionSetpoint = SmartDashboard.getNumber("Set Climber Position (deg)", m_climberPositionSetpoint);
      mt_climberSyncGain = SmartDashboard.getNumber("Set Climber/Sync Gain", mt_climberSyncGain);
      mt_climberMaxDelta = SmartDashboard.getNumber("Set Climber/Max Delta", mt_climberMaxDelta);
    }
}