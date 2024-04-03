// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StagingSubsystem extends SubsystemBase {
  public CANSparkFlex StagingMotor1;
  public CANSparkFlex StagingMotor2;
  /** Creates a new StagingSubsystem. */
  public StagingSubsystem() {
     try{
      StagingMotor1 = new CANSparkFlex(3, MotorType.kBrushless);
    }
    catch(Exception e){
      System.out.println("Missing Bottom Staging Motor");
    }
    try{
      StagingMotor2 = new CANSparkFlex(4, MotorType.kBrushless);
    }
    catch(Exception e){
      System.out.println("Missing Top Staging Motor");
    }
    StagingMotor1.setInverted(true);
  }

  public void DeliverLow(){
    StagingMotor1.set(0.3);
    StagingMotor2.set(0.3);
  }

  public void DeliverHigh(){
    StagingMotor1.set(0.5);
    StagingMotor2.set(0.7);
  }

  public void DeliverStop(){
    StagingMotor1.set(0);
    StagingMotor2.set(0);
  }
  // public void StagingIn(){
  //   StagingMotor1.set(.6);
  //   StagingMotor2.set(-.6);
  // }
  // public void StagingOff(){
  //   StagingMotor1.set(0);
  //   StagingMotor2.set(0);
  // }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
