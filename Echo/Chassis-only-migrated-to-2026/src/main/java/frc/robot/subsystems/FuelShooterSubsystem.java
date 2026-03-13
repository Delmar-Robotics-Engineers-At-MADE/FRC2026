package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Configs;
import frc.robot.Constants.ShooterSubsystemConstants;
import frc.robot.Constants.ShooterSubsystemConstants.FlywheelSetpoints;

public class FuelShooterSubsystem extends SubsystemBase {

   // Flywheel components
   private SparkMax m_motorPort, m_motorStar;
   private SparkClosedLoopController m_flywheelClosedLoopController;
   private RelativeEncoder m_flywheelEncoder;

   // ########### SIM ###########

   DCMotor m_maxSimGearbox = DCMotor.getNEO(2);
   private SparkMaxSim m_motorsSim;
   private FlywheelSim m_flywheelSim = new FlywheelSim(
         LinearSystemId.createFlywheelSystem(m_maxSimGearbox, 0.00316577577, 1), m_maxSimGearbox);

   // ###########################

   private final SendableChooser<Double> m_flywheelVelocityChooser = new SendableChooser<Double>();

   // Member variables for subsystem state management
   private double m_flywheelTargetVelocity = ShooterSubsystemConstants.FlywheelSetpoints.kShootRpm;

   // TEMPORARY: Tuning Constants
   private SparkMaxConfig mt_flywheelConfig = Configs.ShooterSubsystem.flywheelConfig;
   private double mt_flywheelClosedLoopP = 0.0;
   private double mt_flywheelClosedLoopI = 0.0;
   private double mt_flywheelClosedLoopD = 0.0;

   public FuelShooterSubsystem() {

      // Initialize flywheel motors
      m_motorPort = new SparkMax(ShooterSubsystemConstants.kFlywheelMotorCanId, MotorType.kBrushless);
      m_motorStar = new SparkMax(ShooterSubsystemConstants.kFlywheelFollowerMotorCanId, MotorType.kBrushless);

      // Flywheel motors are connected together so use the leader's closed loop
      // controller and encoder for control
      m_flywheelClosedLoopController = m_motorPort.getClosedLoopController();
      m_flywheelEncoder = m_motorPort.getEncoder();

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
      m_motorPort.configure(
            Configs.ShooterSubsystem.flywheelConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
      m_motorStar.configure(
            Configs.ShooterSubsystem.flywheelFollowerConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

      // Zero encoders on initialization
      m_flywheelEncoder.setPosition(0.0);

      // set up sim entities
      m_motorsSim = new SparkMaxSim(m_motorPort, m_maxSimGearbox);

      m_flywheelVelocityChooser.setDefaultOption("Low Speed (1500 rpm)", 1500.0);
      m_flywheelVelocityChooser.addOption("Mid Speed (2500 rpm)", 2500.0);
      m_flywheelVelocityChooser.addOption("Full Speed (4500 rpm)", FlywheelSetpoints.kShootRpm);
      SmartDashboard.putData("DISABLED: Velocity Target", m_flywheelVelocityChooser);

      // TODO: REMOVE LATER AFTER TUNING
      SmartDashboard.putNumber("Set Flywheel Velocity", m_flywheelTargetVelocity);
      SmartDashboard.putNumber("Set Flywheel/kP", mt_flywheelClosedLoopP);
      SmartDashboard.putNumber("Set Flywheel/kI", mt_flywheelClosedLoopI);
      SmartDashboard.putNumber("Set Flywheel/kD", mt_flywheelClosedLoopD);
   }

   private boolean isFlywheelAt(double velocity) {
      return MathUtil.isNear(m_flywheelEncoder.getVelocity(),
            velocity, FlywheelSetpoints.kVelocityTolerance);
   }

   /**
    * Trigger: Is the flywheel spinning at the required velocity?
    */
   public final Trigger isFlywheelSpinning = new Trigger(
         () -> isFlywheelAt(this.m_flywheelTargetVelocity) ||
               m_flywheelEncoder.getVelocity() > this.m_flywheelTargetVelocity);

   public final Trigger isFlywheelSpinningBackwards = new Trigger(
         () -> isFlywheelAt(-this.m_flywheelTargetVelocity) ||
               m_flywheelEncoder.getVelocity() < -this.m_flywheelTargetVelocity);

   /**
    * Trigger: Is the flywheel stopped?
    */
   public final Trigger isFlywheelStopped = new Trigger(() -> isFlywheelAt(0));

   /**
    * Drive the flywheels to their set velocity. This will use MAXMotion
    * velocity control which will allow for a smooth acceleration and deceleration
    * to the mechanism's
    * setpoint.
    */
   private void setFlywheelVelocity(double velocity) {
      m_flywheelClosedLoopController.setSetpoint(velocity, ControlType.kMAXMotionVelocityControl);
   }

   /**
    * Command to run the flywheel motors. When the command is interrupted, e.g. the
    * button is released,
    * the motors will stop.
    */
   public Command runFlywheelCommand() {
      return this.startEnd(
            () -> {
               System.out.println("Spinning Flywheel!!");
               this.setFlywheelVelocity(m_flywheelTargetVelocity);
            },
            () -> {
               m_motorPort.stopMotor();
            }).withName("Spinning Up Flywheel");
   }

   public double getVelocity() {
      return m_flywheelEncoder.getVelocity();
   }

   // TODO: Remove this or move it to a shared place later. There is a matching function in the turret class
   private boolean hasChanged(double a, double b) {
      return Math.abs(a - b) > 1e-6;
   }

   @Override
   public void periodic() {

      //m_flywheelTargetVelocity = m_flywheelVelocityChooser.getSelected(); // TODO: Re-enable this later when tuning is done
      m_flywheelTargetVelocity = SmartDashboard.getNumber("Set Flywheel Velocity", m_flywheelTargetVelocity);

      // Flyhweel attributes
      SmartDashboard.putNumber("Flywheel | Temperature (deg C)", m_motorPort.getMotorTemperature());
      SmartDashboard.putNumber("Flywheel | Applied Output", m_motorPort.getAppliedOutput());
      SmartDashboard.putNumber("Flywheel | Current", m_motorPort.getOutputCurrent());
      SmartDashboard.putNumber("Flywheel | Velocity Controller Setpoint", m_flywheelClosedLoopController.getMAXMotionSetpointVelocity());

      // Flywheel follower attributes
      SmartDashboard.putNumber("Flywheel Follower | Temperature (deg C)", m_motorStar.getMotorTemperature());
      SmartDashboard.putNumber("Flywheel Follower | Applied Output", m_motorStar.getAppliedOutput());
      SmartDashboard.putNumber("Flywheel Follower | Current", m_motorStar.getOutputCurrent());

      // Target velocity vs actual velocity
      SmartDashboard.putNumber("Flywheel | Target Velocity", m_flywheelTargetVelocity);
      SmartDashboard.putNumber("Flywheel | Actual Velocity", m_flywheelEncoder.getVelocity());

      // Track whether the flywheel is spinning (within the tolerance)
      SmartDashboard.putBoolean("Is Flywheel Spinning", isFlywheelSpinning.getAsBoolean());
      SmartDashboard.putBoolean("Is Flywheel Stopped", isFlywheelStopped.getAsBoolean());

      // REMOVE LATER: Tuning PID for the flywheel
      double newkP = SmartDashboard.getNumber("Set Flywheel/kP", mt_flywheelClosedLoopP);
      double newkI = SmartDashboard.getNumber("Set Flywheel/kI", mt_flywheelClosedLoopI);
      double newkD = SmartDashboard.getNumber("Set Flywheel/kD", mt_flywheelClosedLoopD);

      if (hasChanged(newkP, mt_flywheelClosedLoopP)|| hasChanged(newkI, mt_flywheelClosedLoopI) || hasChanged(newkD, mt_flywheelClosedLoopD)) {
         mt_flywheelConfig
            .closedLoop
               .p(newkP)
               .i(newkI)
               .d(newkD);

         m_motorPort.configure(mt_flywheelConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

         mt_flywheelClosedLoopP = newkP;
         mt_flywheelClosedLoopI = newkI;
         mt_flywheelClosedLoopD = newkD;
      }

      // Push current values so they appear on startup
      SmartDashboard.putNumber("Flywheel/kP", mt_flywheelClosedLoopP);
      SmartDashboard.putNumber("Flywheel/kI", mt_flywheelClosedLoopI);
      SmartDashboard.putNumber("Flywheel/kD", mt_flywheelClosedLoopD);
      SmartDashboard.putNumber("Tuning Flywheel Speed", m_flywheelTargetVelocity);
   }

   public void simulationPeriodic() {
      m_flywheelSim.setInput(m_motorPort.getAppliedOutput() * RobotController.getInputVoltage());
      m_flywheelSim.update(0.02);

      // Now, we update the Spark Flex
      m_motorsSim.iterate(
            m_flywheelSim.getAngularVelocityRPM(),
            RobotController.getInputVoltage(), // Simulated battery voltage, in Volts
            0.02); // Time interval, in Seconds

      RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(m_flywheelSim.getCurrentDrawAmps()));
   }
}
