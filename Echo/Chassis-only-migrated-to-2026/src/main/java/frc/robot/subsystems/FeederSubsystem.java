package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.FeederSubsystemConstants;
import frc.robot.Constants.FeederSubsystemConstants.FeederSetpoints;

public class FeederSubsystem extends SubsystemBase {

   // Feeder components
   private SparkMax m_feederMotor;

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
            Configs.ShooterSubsystem.feederConfig,
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
               this.setFeederPower(FeederSetpoints.kFeed);
            }, () -> {
               this.m_feederMotor.stopMotor();;
            }).withName("Feeding");
   }

   @Override
   public void periodic() {

      // Temps
      SmartDashboard.putNumber("Shooter | Feeder | Temperature (deg C)", m_feederMotor.getMotorTemperature());
   }
}
