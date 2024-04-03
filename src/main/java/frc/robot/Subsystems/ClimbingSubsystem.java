// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbingSubsystem extends SubsystemBase {

  public TalonFX ClimbMotorL;
  public TalonFX ClimbMotorR;
  public double ClimbSpeed = .6;

  /** Creates a new ClimbingSubsystem. */
  public ClimbingSubsystem() {
    try{
      ClimbMotorL = new TalonFX(17);
    }
    catch(Exception e){
      System.out.println("Missing Left Climb Motor");
    }
    try{
      ClimbMotorR = new TalonFX(18);
    }
    catch(Exception e){
      System.out.println("Missing Right Climb Motor");
    }
  }
  public void RightCLimb(double RightClimbMotorSpeed){
    double NewRightClimbMotorSpeed = (RightClimbMotorSpeed- (0.3*RightClimbMotorSpeed));
    ClimbMotorR.set(NewRightClimbMotorSpeed);
    
  }
  public void LeftClimb(double LeftClimbMotorSpeed){
    double NewLeftClimbMotorSpeed = (LeftClimbMotorSpeed-(0.3*LeftClimbMotorSpeed));
    ClimbMotorL.set(NewLeftClimbMotorSpeed);
  }
  public void RightCLimbUp(){
    ClimbMotorR.set(-ClimbSpeed);
  }
  public void LeftClimbUp(){
    ClimbMotorL.set(-ClimbSpeed);
  }
  public void DualClimbUp(){
    ClimbMotorR.set(-ClimbSpeed);
    ClimbMotorL.set(-ClimbSpeed);
  }
  public void RightCLimbDown(){
    ClimbMotorR.set(ClimbSpeed);
  } 
  public void LeftClimbDown(){
    ClimbMotorL.set(ClimbSpeed);
  }
  public void DualClimbDown(){
    ClimbMotorL.set(ClimbSpeed);
    ClimbMotorR.set(ClimbSpeed);
  }
  public void RightCLimbStop(){
    ClimbMotorR.set(0);
  }
  public void LeftCLimbStop(){
    ClimbMotorL.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
