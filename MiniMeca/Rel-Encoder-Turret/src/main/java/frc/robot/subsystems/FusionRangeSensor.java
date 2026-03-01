// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

/** This class represents a sensor for detecting april elements of the game field using PhotonVision. */
public final class FusionRangeSensor extends SubsystemBase {

  private final TimeOfFlight m_rangeSensor = new TimeOfFlight(1);

  public FusionRangeSensor() {
    // constructor
    m_rangeSensor.setRangingMode(RangingMode.Short, 40);
    setupDashboard();
  }

  private void setupDashboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("Fusion");
    tab.addInteger("Range(mm)", () -> getRange());
  }  


  public int getRange(){
    // System.out.println("Distance: " + (int)m_rangeSensor.getRange() + "mm   Std Dev: " + (int)m_rangeSensor.getRangeSigma() + "mm   Status: " + m_rangeSensor.getStatus());
    return (int)m_rangeSensor.getRange();
  }

}
