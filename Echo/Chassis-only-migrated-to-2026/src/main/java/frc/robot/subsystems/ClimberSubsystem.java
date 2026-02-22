package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{

   // Climber components
   TalonFX leftClimberMotor = new TalonFX(41);
   TalonFX rightClimberMotor = new TalonFX(42);

   ClimberSubsystem() {

      TalonFXConfiguration leftConfig = new TalonFXConfiguration();
      
      leftConfig
         .MotorOutput
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive);

      leftConfig
         .CurrentLimits
         .withSupplyCurrentLimit(40); // Amps

      leftClimberMotor.setNeutralMode(NeutralModeValue.Brake);
      rightClimberMotor.setNeutralMode(NeutralModeValue.Brake);



      TalonFXConfiguration rightConfig = new TalonFXConfiguration();

      leftConfig.CurrentLimits.withSupplyCurrentLimit(0);

      // TODO: Finish filling out the configuration for the climber motors
   
   }
}