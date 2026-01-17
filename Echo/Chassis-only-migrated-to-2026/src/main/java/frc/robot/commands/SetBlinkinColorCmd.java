// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Blinkin;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class SetBlinkinColorCmd extends Command {
  private final Blinkin m_blinkin;
  private double m_color;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetBlinkinColorCmd(Blinkin blinkin, double color) {
    m_blinkin = blinkin;
    m_color = color;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(blinkin);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // just need to do this once
    m_blinkin.setColor(m_color); 
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // don't need to do anything, as color was set in initialize
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Run until interrupted
  @Override
  public boolean isFinished() {
    return false;
  }
}
