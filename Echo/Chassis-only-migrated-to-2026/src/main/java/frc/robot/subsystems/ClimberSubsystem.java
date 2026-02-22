package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{

   // Climber components
   TalonFX leftClimberMotor = new TalonFX(41);
   TalonFX rightClimberMotor = new TalonFX(42);

   ClimberSubsystem() {

      leftClimberMotor.setNeutralMode(NeutralModeValue.Brake);
      rightClimberMotor.setNeutralMode(NeutralModeValue.Brake);

      TalonFXConfigurator leftConfigurator = leftClimberMotor.getConfigurator();
      TalonFXConfigurator rightConfigurator = rightClimberMotor.getConfigurator();

      TalonFXConfiguration leftConfig = new TalonFXConfiguration();
      TalonFXConfiguration rightConfig = new TalonFXConfiguration();

      leftConfig.CurrentLimits.withSupplyCurrentLimit(0);

      // TODO: Finish filling out the configuration for the climber motors
   
   }
}