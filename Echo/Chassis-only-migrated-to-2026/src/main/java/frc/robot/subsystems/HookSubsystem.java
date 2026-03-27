package frc.robot.subsystems;
import com.revrobotics.ResetMode;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.config.ServoHubConfig;
import com.revrobotics.servohub.config.ServoChannelConfig.BehaviorWhenDisabled;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HookSubsystem extends SubsystemBase {

  private final ServoHub m_servoHub;
  private final ServoChannel m_leftServo;
  private final ServoChannel m_rightServo;

  private static final double REST_PULSE_US = 500;
  private static final double CAPTURE_PULSE_US = 700;  // tune to your capture position in PWM microseconds
  private static final double CURRENT_THRESHOLD_MA = 400; // stall current threshold in mA

  public HookSubsystem(int canId) {
    m_servoHub = new ServoHub(canId);

    ServoHubConfig config = new ServoHubConfig();

    // Configure both channels — adjust ChannelId to your wiring
    config.channel0
        .pulseRange(500, 1500, 2500)  // min, center, max in microseconds
        .disableBehavior(BehaviorWhenDisabled.kDoNotSupplyPower);

    config.channel5
        .pulseRange(500, 1500, 2500)
        .disableBehavior(BehaviorWhenDisabled.kDoNotSupplyPower);

    m_servoHub.configure(config, ResetMode.kResetSafeParameters);

    m_leftServo = m_servoHub.getServoChannel(ChannelId.kChannelId0);
    m_rightServo = m_servoHub.getServoChannel(ChannelId.kChannelId5);

    // Start at rest
    setPulseWidth(REST_PULSE_US);
  }

  public void setPulseWidth(double pulseUs) {
    m_leftServo.setPulseWidth((int) pulseUs);
    m_rightServo.setPulseWidth((int) pulseUs);
  }

  public void disable() {
    m_leftServo.setEnabled(false);
    m_rightServo.setEnabled(false);
  }

  public void enable() {
    m_leftServo.setEnabled(true);
    m_rightServo.setEnabled(true);
  }

  /** True when either servo is drawing above threshold (hitting a physical stop). */
  public boolean isStalled() {
    double leftCurrent = m_leftServo.getCurrent();
    double rightCurrent = m_rightServo.getCurrent();
    // Note: current is reported per BANK (3 channels each), not per channel.
    // If only these two servos are on the bank, this works well.
    return leftCurrent > CURRENT_THRESHOLD_MA || rightCurrent > CURRENT_THRESHOLD_MA;
  }

  /**
   * Command: drive servos to capture position, monitor current,
   * and disable as soon as stall is detected.
   */
  public Command captureHookCommand() {
    return Commands.sequence(
        // Start moving
        Commands.runOnce(() -> {
          enable();
          setPulseWidth(CAPTURE_PULSE_US);
        }, this),

        // Small delay to ignore initial inrush current
        Commands.waitSeconds(0.1),

        // Wait until current spike indicates physical stop
        Commands.waitUntil(this::isStalled),

        // Immediately disable to stop driving into the stop
        Commands.runOnce(this::disable, this)
    ).withName("CaptureHook");
  }
}