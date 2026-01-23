// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.FuelShooterSS;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  static final double TriggerThreshold = 0.5;

  private final FuelShooterSS m_fuelShoot = new FuelShooterSS();
  private final IndexerSubsystem m_indexer = new IndexerSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();

  // for auto driving
//   Alliance m_allianceColor = DriverStation.getAlliance().get();
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  
  Command m_autoDoNothing;

  // Driver
  GenericHID m_driverController = new GenericHID(OIConstants.kDriverControllerPort);
  CommandGenericHID m_driverCmdController = new CommandGenericHID (OIConstants.kDriverControllerPort);
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

    // setup dashboard
    setupDashboard();
  }

  private void buildAutoChooser() {

    // exit staring zone, driving backward toward alliance station
    m_autoDoNothing = new InstantCommand();
    m_autoChooser.setDefaultOption("Exit Start Zone", m_autoDoNothing);
  }

  private void configureNonButtonTriggers() {
  }

  static final int FlightButtonLEFT = 3;
  static final int FlightButtonRIGHT = 4;
  private void configureButtonBindings() {

    // *************************** DRIVER *****************************


    // ******************************** OPERATOR *********************************
  
    // Difference between InstantCommand and RunCommand is the isfinished function.  
    //    for InstantCommand, isFinished is true, so command can only run once
    //    for RunCommand, isFinished is false, so command will run until it is interrupted
    //    RunCommand is good for default command; will run only once, instead of over and over again
    //    InstantCommand is good for onTrue and autonomous sequences
    //    RunCommand is good for whileTrue; will run once, but will get interrupted when whileTrue ends
    m_operCmdController.leftTrigger(TriggerThreshold) // shoot
        .whileTrue(new RunCommand (() -> m_fuelShoot.moveVelocityControl(true, m_operCmdController.getLeftTriggerAxis()), m_fuelShoot)
        .alongWith(new RunCommand (() -> m_indexer.moveVelocityControl(true, m_operCmdController.getLeftTriggerAxis()), m_indexer))
        .alongWith(new RunCommand (() -> m_intake.moveVelocityControl(true, m_operCmdController.getLeftTriggerAxis()), m_indexer)));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  private void setupDashboard() {
    ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
    buildAutoChooser();
    matchTab.add(m_autoChooser).withPosition(0, 0);

  }

  public void checkHomePositions() {
  }
}
