package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FuelShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public final class UtilityCommands {

   // Prevent instantiation; this class is intended only to organize composite command factories
   private UtilityCommands() {}


   /**
    * Meta-command to operate the shooter. The Flywheel starts spinning up and when
    * it reaches
    * the desired speed it starts the Feeder.
    */

   /**
    * Basic shooting command that spins up the flywheel until it is up to speed before feeding in fuel
    * @param shooter The shooter subsystem
    * @param feeder The feeder subsystem
    * @return A simple command that spins the flywheel and feeds fuel at a constant rate
    */
   public static Command runShooterCommand(FuelShooterSubsystem shooter, FeederSubsystem feeder) {
      return Commands.deadline(
               Commands.sequence(
                  Commands.waitUntil(shooter.isFlywheelSpinning),
                  feeder.runFeederCommand()
               ),
               shooter.runFlywheelCommand()
            );
   }

   /**
    * Composite command for shooting at a moving target using a continuously updating position
    * @param turret The turret subsystem; aims the shooter at a target
    * @param feeder The feeder subsystem; loads fuel into the shooter
    * @param shooter The shooter subsystem; launches fuel at the desired target
    * @return A command that enables firing fuel at a particular aimed target
    */
   public static Command shootAtTarget(TurretSubsystem turret, FeederSubsystem feeder, FuelShooterSubsystem shooter) {

      // Deadline will run all of its internal commands until they finish. Since the final command is to run the feeder which
      // will never finish, it will continue until the external caller stops scheduling the command
      return Commands.sequence(
         Commands.deadline(
            Commands.sequence(
               // First, wait until the turret is at the desired position (control of the turret is happening continuously in the "otherCommands")
               Commands.waitUntil(
                  turret.isYawAtPosition.and(turret.isPitchAtPosition) 
               ), 

               // Then, wait for the flywheel to get up to speed (also being commanded to spin up in the "otherCommands")
               Commands.waitUntil(shooter.isFlywheelSpinning),

               // Finally, spin the feeder motor up once everything else is ready
               feeder.runFeederCommand()
            ),

            // Run the flywheel and set the turret and hood positions continuously while the other sequence is occurring
            // TODO: These turret position commands are ignoring the value being passed in currently. Update these once values are actually being passed
            // in (from vision or other)
            shooter.runFlywheelCommand(),
            turret.commandTurretYawToPosition(0),
            turret.commandTurretPitchToPosition(0)
         ),

         // After shooting is done, lower the hood back down
         turret.commandTurretPitchToPosition(0) // TODO: This currently isn't passing anything because the input value is being ignored. Update
                                                         // this once the value can be passed in
      );
   }
}