package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.FeederSubsystemConstants;

public class FeederSubsystem extends SubsystemBase {

   // Feeder components
   private SparkMax m_feederMotor;

   private double mt_feederDutyCycle = Constants.FeederSubsystemConstants.FeederSetpoints.kFeed;

   public FeederSubsystem() {
      
      // Initialize feeder motor
      m_feederMotor = new SparkMax(FeederSubsystemConstants.kFeederMotorCanId, MotorType.kBrushless);

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
      m_feederMotor.configure(
            Configs.FeederSubsystem.feederConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
   }

   /** Set the feeder motor power in the range of [-1, 1]. */
   private void setFeederPower(double power) {
      m_feederMotor.set(power);
   }

   /**
    * Command to run the feeder and flywheel motors. When the command is
    * interrupted, e.g. the button is released,
    * the motors will stop.
    */
   public Command runFeederCommand() {
      return this.startEnd(
            () -> {
               this.setFeederPower(mt_feederDutyCycle);
            }, () -> {
               this.m_feederMotor.stopMotor();;
            }).withName("Feeding");
   }

   @Override
   public void periodic() {

      // REMOVE: Pull the duty cycle value from the dashboard
      mt_feederDutyCycle = SmartDashboard.getNumber("Set Feeder Duty Cycle", mt_feederDutyCycle);

      // Temps
      SmartDashboard.putNumber("Feeder Motor | Temperature (deg C)", m_feederMotor.getMotorTemperature());
      SmartDashboard.putNumber("Feeder Motor | Current (A)", m_feederMotor.getOutputCurrent());
      SmartDashboard.putNumber("Feeder Motor | Applied Output", m_feederMotor.getAppliedOutput());

      // Publish duty cycle config to dashboard
      SmartDashboard.putNumber("Set Feeder Duty Cycle", mt_feederDutyCycle);
   }
}
