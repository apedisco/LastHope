// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShootingSubsystem;
import frc.robot.Subsystems.StagingSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShootHighSeqential extends SequentialCommandGroup {
  /** Creates a new AutoShootHighSeqential. */
  public AutoShootHighSeqential(ShootingSubsystem m_ShootingSubsystem, StagingSubsystem m_StagingSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoShootHighCommand(m_ShootingSubsystem, m_StagingSubsystem));
  }
}
