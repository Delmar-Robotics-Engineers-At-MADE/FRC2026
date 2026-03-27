// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldLocation;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.TurretSubsystemConstants.TurretSetpoints;
import frc.robot.Constants.TurretSubsystemConstants.TurretUnits;
import frc.robot.commands.UtilityCommands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FuelShooterSubsystem;
import frc.robot.subsystems.HookSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.PhotonVisionSensor;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  static final double TriggerThreshold = 0.25;
  static final double PovSpeed = 0.1 * DriveSubsystem.DriveSpeedDivider;  // speed divider slows it down, but we really want this speed not slowed down

  // The robot's subsystems
  private final PhotonVisionSensor m_photon = new PhotonVisionSensor();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_photon);

  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final FeederSubsystem m_feeder = new FeederSubsystem();
  private final TurretSubsystem m_turret = new TurretSubsystem(m_robotDrive.m_odometry);
  private final FuelShooterSubsystem m_fuelShoot = new FuelShooterSubsystem(m_robotDrive.m_odometry);
  private final LightsSubsystem m_lights = new LightsSubsystem();
  private final ClimberSubsystem m_climber = new ClimberSubsystem();
  private final HookSubsystem m_hook = new HookSubsystem(52);

  // TODO: Remove later; tuning constants
  private final double mt_turretYawSetpointDegrees = TurretSetpoints.kYawMotorHomingSetpoint;
  private final double mt_turretPitchSetpointDegrees = TurretSetpoints.kPitchMotorHomingSetpoint;

  // for auto driving
  private final SendableChooser<Command> m_autoChooser;

  // Driver
  GenericHID m_driverController = new Joystick(OIConstants.kDriverControllerPort);
  CommandJoystick m_driverCmdController = new CommandJoystick(OIConstants.kDriverControllerPort);
  // GenericHID m_driverController = new GenericHID(OIConstants.kDriverControllerPort);
  // CommandGenericHID m_driverCmdController = new CommandGenericHID (OIConstants.kDriverControllerPort);

  // Operator
  XboxController m_operController = new XboxController(OIConstants.kOperatorControllerPort);
  CommandXboxController  m_operCmdController = new CommandXboxController (OIConstants.kOperatorControllerPort);

  // Button pad (PXN)
  GenericHID m_buttonPad = new GenericHID(OIConstants.kButtonPadPort);
  CommandGenericHID  m_buttonPadCmd = new CommandGenericHID (OIConstants.kButtonPadPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    // Configure non-button triggers
    configureNonButtonTriggers();
    m_autoChooser = AutoBuilder.buildAutoChooser();
    // setup dashboard
    setupDashboard();

    setupCommands();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(-m_driverController.getRawAxis(1), OIConstants.kDriveDeadband), // getLeftY()
                MathUtil.applyDeadband(-m_driverController.getRawAxis(0), OIConstants.kDriveDeadband), // getLeftX()
                -MathUtil.applyDeadband(m_driverController.getRawAxis(2), OIConstants.kDriveDeadband*4), // getRightX()
                true),
            m_robotDrive));

    // Brake the rotation of the turret by default unless the turret has not been homed yet
    // Also make sure the hood is constantly down unless being actively commanded
    // and that it is home immediately
    m_turret.setDefaultCommand(
      Commands.either(
        m_turret.setTurretIdle(),
        m_turret.homeFullTurret(() -> m_operCmdController.getRightX()),
        m_turret.isTurretFullyHomed())
    );

    m_lights.test();

    // TODO: REMOVE LATER: Tuning PID turret
    SmartDashboard.putNumber("Set Turret Yaw Position", mt_turretYawSetpointDegrees);
    SmartDashboard.putNumber("Set Turret Pitch Position", mt_turretPitchSetpointDegrees);
    SmartDashboard.putNumber("Set Turret Yaw FF", TurretUnits.kYawFF);
  }


  private void configureNonButtonTriggers() {
  }

  static final int FlightButtonTRIGGER = 1;
  static final int FlightButtonTHUMB = 2;
  static final int FlightButtonLEFT = 3;
  static final int FlightButtonRIGHT = 4;
  static final int FlightButtonUPPERLEFT = 5;
  static final int FlightButtonUPPERRIGHT = 6;
  private void configureButtonBindings() {

    // ******************************** OPERATOR *********************************
  
    // reset pose to vision
    // m_operCmdController.back().or(m_operCmdController.start())
    //     .and(m_operCmdController.y())
    //     .onTrue(new InstantCommand (() -> m_robotDrive.debugResetOdometryToVision(m_photon), m_robotDrive, m_photon));

    // // Driver Trigger -> Shoot! By default at hub, or left or right offense zones with extra button press
    // new JoystickButton(m_driverController, FlightButtonTRIGGER) // thumb button on flight controller
    //      .whileTrue(UtilityCommands.runShooterCommand(m_fuelShoot, m_feeder, m_turret, m_intake, FieldLocation.HUB.getPosition()));

    // new JoystickButton(m_driverController, FlightButtonLEFT) // thumb button on flight controller
    //     .onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

    // Trigger alone (no modifiers held) → shoot at default target (e.g. HUB)
    m_driverCmdController.trigger()
        .and(m_driverCmdController.button(FlightButtonLEFT).negate())
        .and(m_driverCmdController.button(FlightButtonRIGHT).negate())
        .whileTrue(UtilityCommands.runShooterCommand(m_fuelShoot, m_feeder, m_turret, m_intake, FieldLocation.HUB.getPosition()));

    // Trigger + button 3 (left bottom) → shoot at Left corner of own alliance zone from driver perspective
    m_driverCmdController.trigger()
        .and(m_driverCmdController.button(FlightButtonLEFT))
        .whileTrue(UtilityCommands.runShooterCommand(m_fuelShoot, m_feeder, m_turret, m_intake, FieldLocation.LEFT_CORNER.getPosition()));

    // Trigger + button 4 → shoot at Right corner of own alliance zone from driver perspective
    m_driverCmdController.trigger()
        .and(m_driverCmdController.button(FlightButtonRIGHT))
        .whileTrue(UtilityCommands.runShooterCommand(m_fuelShoot, m_feeder, m_turret, m_intake, FieldLocation.RIGHT_CORNER.getPosition()));

    // Top right button on flight stick; runs intake
    m_driverCmdController.button(FlightButtonUPPERRIGHT)
      .whileTrue(m_intake.runCombinedCommand());

    // A button -> run the intake 
    m_operCmdController.a().whileTrue(m_intake.runIntakeCommand());

    // D-pad down; mirror of 'a' button -> run the intake in reverse
    m_operCmdController.povDown().whileTrue(m_intake.runExtakeCommand());

    // B button -> run the roller floor in reverse
    m_operCmdController.b().whileTrue(m_intake.runConveyerCommand());

    // D-pad left; mirror of 'b' button -> run the intake and roller floor in reverse
    m_operCmdController.povLeft().whileTrue(m_intake.runConveyerReverseCommand());

    // Y button -> run the feeder forwards
    m_operCmdController.y().whileTrue(m_feeder.runFeederCommand());

    // D-pad up; mirror of 'y' button -> run the feeder in reverse
    m_operCmdController.povUp().whileTrue(m_feeder.runFeederReverseCommand());

    m_operCmdController.x().onTrue(m_turret.commandTurretPitchToPosition(() -> TurretSetpoints.kPitchMotorMaxSetpoint));

    // Alternate shooting trigger to mirror at hub ability of driver controller
    m_operCmdController.rightTrigger(TriggerThreshold)
      .whileTrue(UtilityCommands.runShooterCommand(m_fuelShoot, m_feeder, m_turret, m_intake, FieldLocation.HUB.getPosition()));
    
    // // A button -> Spin feeder/loader motor into shooter
    // m_operCmdController.a().whileTrue(m_feeder.runFeederCommand());

    // // B button -> Turn turret pitch to a set point
    // // TODO: Update this later after testing its movement; it is currently using a member variable that is editable in the dashboard
    // m_operCmdController.b().whileTrue(m_turret.commandTurretPitchToPosition(() -> SmartDashboard.getNumber("Set Turret Pitch Position", mt_turretPitchSetpointDegrees)));

    // // X button -> turn turret yaw to a set point
    // // TODO: Update this later after testing its movement; it is currently using a member variable that is editable in the dashboard
    // m_operCmdController.x().whileTrue(m_turret.commandTurretYawToPosition(SmartDashboard.getNumber("Set Turret Yaw Position", mt_turretYawSetpointDegrees)));

    // m_operCmdController.y().whileTrue(m_turret.trackHubCommand(FieldLocation.HUB.getPosition()));

    m_operCmdController.rightBumper().whileTrue(m_climber.moveArmsUpCommand());
    m_operCmdController.leftBumper().whileTrue(m_climber.moveArmsDownCommand());
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  private void setupDashboard() {
    // ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
    //matchTab.addCamera("Limelight", "Limelight", "http://10.80.77.18:5800");
    //        .withPosition(0, 1).withSize(4, 3);
    if (m_autoChooser != null)
    {
      SmartDashboard.putData("Auto Chooser", m_autoChooser);
    }

  }

  private void setupCommands() {
      NamedCommands.registerCommand(
          "Shoot",
          UtilityCommands
              .runShooterCommand(m_fuelShoot, m_feeder, m_turret, m_intake, FieldLocation.HUB.getPosition())
              .withTimeout(5.0)
      );

      NamedCommands.registerCommand(
          "Intake",
          m_intake.runCombinedCommand().withTimeout(5.0)
      );
  }
}
