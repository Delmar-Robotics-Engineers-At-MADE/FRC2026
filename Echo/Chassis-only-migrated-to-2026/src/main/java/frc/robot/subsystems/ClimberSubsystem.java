package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberSubsystemConstants;

public class ClimberSubsystem extends SubsystemBase{

   // Climber components
   TalonFX leftClimberMotor = new TalonFX(ClimberSubsystemConstants.kClimberLeftMotorCanId);
   TalonFX rightClimberMotor = new TalonFX(ClimberSubsystemConstants.kClimberRightMotorCanId);

   ClimberSubsystem() {

      TalonFXConfiguration leftConfig = new TalonFXConfiguration();
      TalonFXConfiguration rightConfig = new TalonFXConfiguration();
      
      leftClimberMotor.getConfigurator().apply(leftConfig);
      rightClimberMotor.getConfigurator().apply(rightConfig);
      
      // TODO: Finish filling out the configuration for the climber motors
   
   }
}