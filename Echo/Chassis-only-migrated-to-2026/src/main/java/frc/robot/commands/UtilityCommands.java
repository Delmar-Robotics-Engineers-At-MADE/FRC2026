package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.TurretSubsystemConstants.TurretSetpoints;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FuelShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public final class UtilityCommands {

   // Prevent instantiation; this class is intended only to organize composite command factories
   private UtilityCommands() {}

   /**
    * Basic shooting command that spins up the flywheel until it is up to speed before feeding in fuel
    * @param shooter The shooter subsystem
    * @param feeder The feeder subsystem
    * @return A simple command that spins the flywheel and feeds fuel at a constant rate
    */
   public static Command runShooterCommand(FuelShooterSubsystem shooter, FeederSubsystem feeder, 
                                           TurretSubsystem turret, IntakeSubsystem intake, Translation2d targetPos) {
      return Commands.deadline(
               Commands.sequence(
                  Commands.waitUntil(shooter.isFlywheelSpinning),
                  Commands.waitUntil(turret.isYawAtTargetPosition()),
                  Commands.parallel(
                     feeder.runFeederCommand(),
                     intake.runCombinedCommand()
                  )
               ),
               shooter.trackHubCommand(targetPos),
               turret.trackHubCommand(targetPos)
            ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
   }

//    /**
//     * Composite command for shooting at a moving target using a continuously updating position
//     * @param turret The turret subsystem; aims the shooter at a target
//     * @param feeder The feeder subsystem; loads fuel into the shooter
//     * @param shooter The shooter subsystem; launches fuel at the desired target
//     * @return A command that enables firing fuel at a particular aimed target
//     */
//    public static Command shootAtTarget(TurretSubsystem turret, FeederSubsystem feeder, FuelShooterSubsystem shooter, Supplier<Pose2d> poseSupplier,
//                                        Translation2d targetPos) {

//       // Deadline will run all of its internal commands until they finish. Since the final command is to run the feeder which
//       // will never finish, it will continue until the external caller stops scheduling the command
//       return Commands.deadline(
//                Commands.sequence(
//                      // First, wait until the turret is at the desired position (control of the turret is happening continuously in the "otherCommands")
//                   Commands.waitUntil(
//                      () -> turret.isYawAtTargetPosition().getAsBoolean() && (turret.isPitchAtPosition(40.0).getAsBoolean()) 
//                   ), 

//                   // Then, wait for the flywheel to get up to speed (also being commanded to spin up in the "otherCommands")
//                   Commands.waitUntil(shooter.isFlywheelSpinning),

//                   // Finally, spin the feeder motor up once everything else is ready
//                   feeder.runFeederCommand()
//                ),

//                // Run the flywheel and set the turret and hood positions continuously while the other sequence is occurring
//                shooter.trackHubCommand(targetPos),
//                turret.trackHubCommand(targetPos)
//             );
//    }
}