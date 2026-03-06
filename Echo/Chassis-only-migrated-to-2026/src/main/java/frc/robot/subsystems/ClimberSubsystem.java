package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{

   // Climber components
   TalonFX leftClimberMotor = new TalonFX(41);
   TalonFX rightClimberMotor = new TalonFX(42);

   ClimberSubsystem() {

      TalonFXConfiguration leftConfig = new TalonFXConfiguration();
      TalonFXConfiguration rightConfig = new TalonFXConfiguration();

      leftConfig
         .Feedback
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

      leftConfig
         .MotorOutput
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive);

      leftConfig
         .OpenLoopRamps
            .withDutyCycleOpenLoopRampPeriod(0.1);

      leftConfig
         .CurrentLimits
            .withSupplyCurrentLimit(40)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLowerLimit(30)
            .withSupplyCurrentLowerTime(1.5)
            .withStatorCurrentLimit(60)
            .withStatorCurrentLimitEnable(true); // Amps

      rightConfig = leftConfig.clone();
      
      leftClimberMotor.getConfigurator().apply(leftConfig);
      rightClimberMotor.getConfigurator().apply(rightConfig);
      
      // TODO: Finish filling out the configuration for the climber motors
   
   }
}