// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix6.Utils;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

public class AutoTest extends Command {
  CommandSwerveDrivetrain m_CommandSwerveDrivetrain;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
  private Timer m_time = new Timer();
  
  /** Creates a new AutoTest. */
  public AutoTest(CommandSwerveDrivetrain commandSwerveDrivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_CommandSwerveDrivetrain = commandSwerveDrivetrain;

    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_time.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if(m_time.get() < 6.7){
        m_CommandSwerveDrivetrain.setControl( drive.withVelocityX(0 ) // Drive forward with
            // negative Y (forward)
            .withVelocityY(0 ) // Drive left with negative X (left)
            .withRotationalRate(.6) // Drive counterclockwise with negative X (left)
        );
      }

      // if(m_time.get() > 6.7 && m_time.get()< 10){
      //   m_CommandSwerveDrivetrain.setControl( drive.withVelocityX(.4 ) // Drive forward with
      //       // negative Y (forward)
      //       .withVelocityY(0 ) // Drive left with negative X (left)
      //       .withRotationalRate(0) // Drive counterclockwise with negative X (left)
      //   );
      // }
      
      else {
        m_CommandSwerveDrivetrain.setControl( drive.withVelocityX(0 ) // Drive forward with
            // negative Y (forward)
            .withVelocityY(0 ) // Drive left with negative X (left)
            .withRotationalRate(0) // Drive counterclockwise with negative X (left)
        );
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

     m_CommandSwerveDrivetrain.applyRequest(() -> drive.withVelocityX(0 ) // Drive forward with
            // negative Y (forward)
            .withVelocityY(0 ) // Drive left with negative X (left)
            .withRotationalRate(0) // Drive counterclockwise with negative X (left)
        );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}