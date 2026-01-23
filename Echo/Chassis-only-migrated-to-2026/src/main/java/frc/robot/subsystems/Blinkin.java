// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SetBlinkinColorCmd;

// Allows control of fun LED lights if Ryan doesn't rip them off before competition
public class Blinkin extends SubsystemBase {

  public static final class LEDConstants {
    public static final double green = 0.77;
    public static final double purple = 0.91;
    public static final double red = -0.31;
    public static final double blue = -0.29;
    public static final double grey = -0.33;
  }
  
  // private static Blinkin instance = null;
  // public Blinkin getInstance() {
  //   if (instance == null) {
  //     instance = new Blinkin();
  //   }
  //   else {
  //     instance = this;
  //   }
  //   return instance;
  // }
  private final Spark one;
  private final Spark two;

  /** Creates a new Blinkin. */
  public Blinkin() {
    one = new Spark(0);
    two = new Spark(1);
    if(DriverStation.getAlliance().get() == Alliance.Red) {
      setDefaultCommand(new SetBlinkinColorCmd(this, LEDConstants.red));
    } else {
      setDefaultCommand(new SetBlinkinColorCmd(this, LEDConstants.blue));
    }
}

  public void setColor(double input) {
    one.set(input);
    two.set(input);
  }

  // public void setAllianceColor() {
    
  //   if(m_allianceColor == Alliance.Red) {
  //     setColor(LEDConstants.red);
  //   } else {
  //     setColor(LEDConstants.blue);
  //   }
  // }

  // public Command setAllianceColorCmd() {
  //   return runOnce(() -> setAllianceColor());
  // }

  // public Command setCmd(double color) {
  //   return new RunCommand(() -> setColor(color), this);
  // }

  // public void setDefault() {
  //     Optional<Alliance> alliance = DriverStation.getAlliance();
  //     if(alliance.get() == Alliance.Red) {
  //       super.setDefaultCommand(redCmd());
  //     }
  //     else {
  //       super.setDefaultCommand(blueCmd());
  //     }
  // }

  // public Command indCapture() {
  //   return new SequentialCommandGroup(
  //     set(LEDConstants.green),
  //     new WaitCommand(0.2),
  //     set(LEDConstants.grey),
  //     new WaitCommand(0.2),
  //     set(LEDConstants.green),
  //     new WaitCommand(0.2)
  //   );
  // }

  // public Command purpleCmd() {
  //   return setCmd(LEDConstants.purple);
  // }

  // public Command greenCmd() {
  //   return setCmd(LEDConstants.green);
  // }

  // public Command redCmd() {
  //   return setCmd(LEDConstants.red);
  // }

  // public Command blueCmd() {
  //   return setCmd(LEDConstants.blue);
  // }
}
