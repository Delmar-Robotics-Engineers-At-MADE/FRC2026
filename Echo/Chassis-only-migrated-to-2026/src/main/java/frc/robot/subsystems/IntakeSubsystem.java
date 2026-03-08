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
import frc.robot.Constants.IntakeSubsystemConstants;
import frc.robot.Constants.IntakeSubsystemConstants.ConveyorSetpoints;
import frc.robot.Constants.IntakeSubsystemConstants.IntakeSetpoints;

public class IntakeSubsystem extends SubsystemBase{

  private SparkMax m_intakeMotor = new SparkMax(IntakeSubsystemConstants.kIntakeMotorCanId, MotorType.kBrushless);
  private SparkMax m_conveyorMotor = new SparkMax(IntakeSubsystemConstants.kConveyorMotorCanId, MotorType.kBrushless);

  private double mt_intakeDutyCycle = Constants.IntakeSubsystemConstants.IntakeSetpoints.kIntake;
  private double mt_conveyorDutyCycle = Constants.IntakeSubsystemConstants.ConveyorSetpoints.kIntake;

  public IntakeSubsystem() {
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
    m_intakeMotor.configure(
        Configs.IntakeSubsystem.intakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_conveyorMotor.configure(
      Configs.IntakeSubsystem.conveyorConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    System.out.println("---> IntakeSubsystem initialized");
  }

  /** Set the intake motor power in the range of [-1, 1]. */
  private void setIntakePower(double power) {
    m_intakeMotor.set(power);
  }

  /** Set the conveyor motor power in the range of [-1, 1]. */
  private void setConveyorPower(double power) {
    m_conveyorMotor.set(power);
  }

  /**
   * Command to run the intake and conveyor motors. When the command is interrupted, e.g. the button is released,
   * the motors will stop.
   */
  public Command runIntakeCommand() {
    return this.startEnd(
        () -> {
          this.setIntakePower(mt_intakeDutyCycle);
          this.setConveyorPower(mt_conveyorDutyCycle);
        }, () -> {
          this.setIntakePower(0.0);
          this.setConveyorPower(0.0);
        }).withName("Intaking");
  }

  /**
   * Command to reverse the intake motor and coveyor motors. When the command is interrupted, e.g. the button is
   * released, the motors will stop.
   */
  public Command runExtakeCommand() {
    return this.startEnd(
        () -> {
          this.setIntakePower(IntakeSetpoints.kExtake);
          this.setConveyorPower(ConveyorSetpoints.kExtake);
        }, () -> {
          this.setIntakePower(0.0);
          this.setConveyorPower(0.0);
        }).withName("Extaking");
  }

  @Override
  public void periodic() {

    mt_intakeDutyCycle = SmartDashboard.getNumber("Set Intake Duty Cycle", mt_intakeDutyCycle);
    mt_conveyorDutyCycle = SmartDashboard.getNumber("Set Conveyor Duty Cycle", mt_conveyorDutyCycle);

    // Intake Values
    SmartDashboard.putNumber("Intake | Temperature (deg C)", m_intakeMotor.getMotorTemperature());
    SmartDashboard.putNumber("Intake | Current", m_intakeMotor.getOutputCurrent());
    SmartDashboard.putNumber("Intake | Applied Output", m_intakeMotor.getAppliedOutput());
    
    // Conveyor Values
    SmartDashboard.putNumber("Conveyor | Temperature (deg C)", m_conveyorMotor.getMotorTemperature());
    SmartDashboard.putNumber("Conveyor | Current", m_conveyorMotor.getOutputCurrent());
    SmartDashboard.putNumber("Conveyor | Applied Output", m_conveyorMotor.getAppliedOutput());

    // Publish duty cycles back to the dashboard
    SmartDashboard.putNumber("Set Intake Duty Cycle", mt_intakeDutyCycle);
    SmartDashboard.putNumber("Set Conveyor Duty Cycle", mt_conveyorDutyCycle);
  }

}
