// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;




public class IntakeSubsystem extends SubsystemBase {
  public CANSparkFlex IntakeMotor1;
  public CANSparkFlex IntakeMotor2;
  // public WPI_TalonSRX NoteLightsMotor;

  public double MaxSpeed = 1;
  public double IntakeSpeed = .2;
  public double StagingIntakeSpeed = .15;
  public double StagingReverseSpeed = .15;
  public double ShootLowerRollerSpeed = .25;
  public double ShootUpperRollerSpeed = .55;
  public double Shooot= .4;
  //Creats the Break Beam sensor and gives it the DIO port;
  // public DigitalInput StagingSensor = new DigitalInput(0);
  public double engage_time;
  //public DigitalInput StagingSensor;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
  //Checks to see if the motor is connected and if it isnt it prints a line to the terminal;
    try{
      IntakeMotor1 = new CANSparkFlex(1, MotorType.kBrushless);
    }
    catch(Exception e){
      System.out.println("Missing Left Shooting Motor");
    }
    try{
      IntakeMotor2 = new CANSparkFlex(2, MotorType.kBrushless);
    }
    catch(Exception e){
      System.out.println("Missing Right Shooting Motor");
    }
  }

  //Creates classes that are called by the commands;
    
    
    public void Intake(){
      IntakeMotor1.set(IntakeSpeed);
      IntakeMotor2.set(-IntakeSpeed);
    }
    public void Outtake(){
      IntakeMotor1.set(0);
      IntakeMotor2.set(0);
    }

    public void set_time(){
      engage_time = System.currentTimeMillis();
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
