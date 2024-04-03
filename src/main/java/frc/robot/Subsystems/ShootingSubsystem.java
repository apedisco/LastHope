// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShootingSubsystem extends SubsystemBase {
  public CANSparkFlex ShootingMotor1;
  public SparkPIDController pidController1;
  public RelativeEncoder shooterEncoder1;

  public CANSparkFlex ShootingMotor2;
  public SparkPIDController pidController2;
  public RelativeEncoder shooterEncoder2;
  /** Creates a new ShootingSubsystem. */
  public ShootingSubsystem() {
     try{
      ShootingMotor1 = new CANSparkFlex(5, MotorType.kBrushless);
    }
    catch(Exception e){
      System.out.println("Missing Right Staging Motor");
    }
    try{
      ShootingMotor2 = new CANSparkFlex(6, MotorType.kBrushless);
    }
    catch(Exception e){
      System.out.println("Missing Left Staging Motor");
    }
    ShootingMotor1.setIdleMode(IdleMode.kCoast);
    ShootingMotor2.setIdleMode(IdleMode.kCoast);
    pidController2 = ShootingMotor2.getPIDController();
    pidController2.setP(0.00025);
    pidController2.setI(0.0);
    pidController2.setD(0.0);

    shooterEncoder2 = ShootingMotor2.getEncoder();

    pidController1 = ShootingMotor1.getPIDController();
    pidController1.setP(0.0005);
    pidController1.setI(0.0);
    pidController1.setD(0.1);

    shooterEncoder1 = ShootingMotor1.getEncoder();
    ShootingMotor1.setInverted(true);
  }

  public void Rev(double motorSpeed){
    ShootingMotor1.set(motorSpeed);
    ShootingMotor2.set(motorSpeed);
  }
  
  public void pidShooting1(double setpoint){
    pidController1.setReference(setpoint, CANSparkFlex.ControlType.kVelocity);
  }
  public void pidShooting2(double stetpoint){
    pidController2.setReference(stetpoint, CANSparkFlex.ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter1 Speed", shooterEncoder1.getVelocity());
    SmartDashboard.putNumber("Shooter2 Speed", shooterEncoder2.getVelocity());
  }
}
