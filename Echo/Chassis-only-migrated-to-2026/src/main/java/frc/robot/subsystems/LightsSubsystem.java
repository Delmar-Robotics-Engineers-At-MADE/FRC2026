package frc.robot.subsystems;

import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.hardware.CANdle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.LightsSubsystemConstants;

public class LightsSubsystem extends SubsystemBase {

   private final CANdle m_candle;

   public LightsSubsystem() {
      m_candle = new CANdle(LightsSubsystemConstants.kCANdleCanId);

      m_candle.getConfigurator().apply(Configs.CANdleSubsystem.config);

      // Set up a rainbow animation to flow through the connected LEDs
      RainbowAnimation rainbowAnimation = new RainbowAnimation(8, 24).withSlot(0);

      // Set up a fire animation for the CANdle itself's built in LEDs
      FireAnimation fireAnimation = new FireAnimation(0, 7).withSlot(1);

      m_candle.setControl(rainbowAnimation);
      m_candle.setControl(fireAnimation);
   }

   public void test()
   {
      System.out.println("Lights active!");
   }
}
