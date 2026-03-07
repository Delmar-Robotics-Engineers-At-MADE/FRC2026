package frc.robot.subsystems;

import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.hardware.CANdle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.LightsSubsystemConstants;

public class LightsSubsystem extends SubsystemBase {

   private final CANdle m_candle;

   public LightsSubsystem() {
      m_candle = new CANdle(LightsSubsystemConstants.kCANdleCanId);

      m_candle.getConfigurator().apply(Configs.CANdleSubsystem.config);

      // Trigger an animation to flow through the connected LEDs
      ColorFlowAnimation colorFlow = new ColorFlowAnimation(8, 24);
      m_candle.setControl(colorFlow);
   }
}
