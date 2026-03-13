package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ClimberSubsystemConstants;

public class ClimberSubsystem extends SubsystemBase{

   // Climber components
   TalonFX leftClimberMotor = new TalonFX(ClimberSubsystemConstants.kClimberLeftMotorCanId);
   TalonFX rightClimberMotor = new TalonFX(ClimberSubsystemConstants.kClimberRightMotorCanId);

   ClimberSubsystem() {

      TalonFXConfiguration leftConfig = Configs.ClimberSubsystem.leftConfig;
      TalonFXConfiguration rightConfig = Configs.ClimberSubsystem.rightConfig;
      
      leftClimberMotor.getConfigurator().apply(leftConfig);
      rightClimberMotor.getConfigurator().apply(rightConfig);
   }

   private void moveLeftArm(double position) {
      MotionMagicDutyCycle command = new MotionMagicDutyCycle(position).withSlot(0);
      leftClimberMotor.setControl(command);
   }
}