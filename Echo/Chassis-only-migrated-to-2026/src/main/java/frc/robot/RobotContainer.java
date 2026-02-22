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
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FuelShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonVisionSensor;
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

  static final double TriggerThreshold = 0.25;
  static final double PovSpeed = 0.1 * DriveSubsystem.DriveSpeedDivider;  // speed divider slows it down, but we really want this speed not slowed down

  // The robot's subsystems
  //private final PhotonVisionSensor m_photon = new PhotonVisionSensor();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  private final FuelShooterSubsystem m_fuelShoot = new FuelShooterSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();

  // TODO: Add fun LEDs back in if time and weight permit
  //private final Blinkin m_blinkin = new Blinkin();

  // for auto driving
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

  }

  public void simulationPeriodic() {
    // Update the simulation model.
    m_fuelShoot.simulationPeriodic();
  }

  // private Command driveToAprilTagCommand (int id, HornSelection hornSelect) {
  //   // System.out.println("New command to tag " + id);
  //   return m_robotDrive.setTrajectoryToAprilTargetCmd(id, hornSelect, m_photon)
  //   .andThen(m_robotDrive.getSwerveControllerCmdForTeleop(m_photon))
  //   .andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  // }

  // private Command rotateDownfieldCommand () {
  //   return m_robotDrive.setTrajectoryToRotateDownfieldCmd(m_photon)
  //   .andThen(m_robotDrive.getSwerveControllerCmdForTeleop(m_photon))
  //   .andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  // }

  // int[] redReefPositionToAprilTag = {0, 11, 10, 9, 6, 7, 8};
  // private Command driveToReefPositionCmd (int pos, HornSelection hornSelect) {
  //       return driveToAprilTagCommand (redReefPositionToAprilTag[pos], hornSelect);
  // }

  private void buildAutoChooser() {

    // exit staring zone, driving backward toward alliance station
    m_autoDoNothing = new InstantCommand();
    m_autoChooser.setDefaultOption("Exit Start Zone", m_autoDoNothing);
  }

  private void configureNonButtonTriggers() {
  }

  static final int FlightButtonTHUMB = 2;
  static final int FlightButtonLEFT = 3;
  static final int FlightButtonRIGHT = 4;
  private void configureButtonBindings() {

    // *************************** DRIVER *****************************

    new JoystickButton(m_driverController, FlightButtonTHUMB) // thumb button on flight controller
        .onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

    // reef positions
    // m_buttonPadCmd.button(3).and(m_photon::getPoseEstimateAcquired)
    //     .whileTrue(driveToReefPositionCmd(4, HornSelection.Between));

    // Coral stations and Processor
    // m_buttonPadCmd.button(1).and(m_photon::getPoseEstimateAcquired) // processor
    //     .whileTrue(driveToCoralStationCmd(HornSelection.L));

    // drive robot relative in cardinal directions
    m_buttonPadCmd.povUp().whileTrue(new RunCommand(
        () -> m_robotDrive.drive(PovSpeed, 0, 0, false), m_robotDrive));
    m_buttonPadCmd.povDown().whileTrue(new RunCommand(
        () -> m_robotDrive.drive(-PovSpeed, 0, 0, false), m_robotDrive));
    m_buttonPadCmd.povLeft().whileTrue(new RunCommand(
        () -> m_robotDrive.drive(0, PovSpeed, 0, false), m_robotDrive));
    m_buttonPadCmd.povRight().whileTrue(new RunCommand(
        () -> m_robotDrive.drive(0, -PovSpeed, 0, false), m_robotDrive));
                    
    // ******************************** OPERATOR *********************************
  
    // reset pose to vision
    // m_operCmdController.back().or(m_operCmdController.start())
    //     .and(m_operCmdController.y())
    //     .onTrue(new InstantCommand (() -> m_robotDrive.debugResetOdometryToVision(m_photon), m_robotDrive, m_photon));

    // Right Trigger -> Run fuel intake
    m_operCmdController
      .rightTrigger(TriggerThreshold)
        .whileTrue(m_intake.runIntakeCommand());

    // Left Trigger -> Spin shooterflywheel
    m_operCmdController
      .leftTrigger(TriggerThreshold)
        .whileTrue(m_fuelShoot.runFlywheelCommand());

    // A button -> Spin feeder/loader motor into shooter
    m_operCmdController.a().whileTrue(m_fuelShoot.runFeederCommand());

    // Left stick movement along the X axis control the turret rotational movement
    m_operCmdController
      .axisMagnitudeGreaterThan(0, TriggerThreshold)
        .whileTrue(m_fuelShoot.moveTurretRotationManual(() -> this.m_operCmdController.getLeftX()));

    // Left stick movement along the y axis contrtols the turret hood movement
    m_operCmdController
      .axisMagnitudeGreaterThan(1, TriggerThreshold)
        .whileTrue(m_fuelShoot.moveTurretHoodManual(() -> this.m_operCmdController.getLeftY()));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  private void setupDashboard() {
    ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
    matchTab.addCamera("Limelight", "Limelight", "http://10.80.77.18:5800");
//         .withPosition(0, 1).withSize(4, 3);
    buildAutoChooser();
    matchTab.add(m_autoChooser).withPosition(0, 0);

  }

  public void checkHomePositions() {
  }
}
