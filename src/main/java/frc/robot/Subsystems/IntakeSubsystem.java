// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Telemetry;
import frc.robot.Commands.TestCommands.ClimbCommands.RightCLimbUpCommand;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
//import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;



public class IntakeSubsystem extends SubsystemBase {
  public CANSparkFlex IntakeMotor1;
  public CANSparkFlex IntakeMotor2;
  public CANSparkFlex StagingMotor1;
  public CANSparkFlex StagingMotor2;
  public CANSparkFlex ShootingMotor1;
  public CANSparkFlex ShootingMotor2;
  public TalonFX ClimbMotorL;
  public TalonFX ClimbMotorR;
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

  //StagingSensor = Robot.MasterStagingSensor;
     
  // ClimbMotorL.setInverted(True);
  //Creates classes that are called by the commands;
    public void RightCLimb(double RightClimbMotorSpeed){
      double NewRightClimbMotorSpeed = (RightClimbMotorSpeed- (0.3*RightClimbMotorSpeed));
      ClimbMotorR.set(NewRightClimbMotorSpeed);
    }
    public void LeftClimb(double LeftClimbMotorSpeed){
      double NewLeftClimbMotorSpeed = (LeftClimbMotorSpeed-(0.3*LeftClimbMotorSpeed));
      ClimbMotorL.set(NewLeftClimbMotorSpeed);
    }
    public void NoteDumpOn(){
      IntakeMotor1.set(1);
      IntakeMotor2.set(-1);
      StagingMotor1.set(-1);
      StagingMotor2.set(-1);
    }
    public void NoteDumpOff(){
      IntakeMotor1.set(0);
      IntakeMotor2.set(0);
      StagingMotor1.set(0);
      StagingMotor2.set(0);
    }
    public void AntiNote(){
      IntakeMotor1.set(1);
    }
    public void RightCLimbUp(){
      ClimbMotorR.set(-1);
    }
    public void LeftClimbUp(){
      ClimbMotorL.set(-1.);
    }
    public void DualClimbUp(){
      ClimbMotorR.set(-1);
      ClimbMotorL.set(-1);
    }
    public void RightCLimbDown(){
      ClimbMotorR.set(1);
    } 
    public void LeftClimbDown(){
      ClimbMotorL.set(1);
    }
    public void DualClimbDown(){
      ClimbMotorL.set(1);
      ClimbMotorR.set(1);
    }
    public void RightCLimbStop(){
      ClimbMotorR.set(0);
    }
    public void LeftCLimbStop(){
      ClimbMotorL.set(0);
    }
    public void Intake(){
      IntakeMotor1.set(.2);
      IntakeMotor2.set(.2);
    }
    public void Outtake(){
      IntakeMotor1.set(0);
      IntakeMotor2.set(0);
    }
    public void IntakeIn(){

      IntakeMotor1.set(.2);
      IntakeMotor2.set(.2);
      StagingMotor1.set(-.15);
      StagingMotor2.set(.15);
    }
    public void IntakeOff(){ 
      IntakeMotor1.set(0);
      IntakeMotor2.set(0);
      StagingMotor1.set(0);
      StagingMotor2.set(0);
    }
    public void set_time(){
      engage_time = System.currentTimeMillis();
    }
    public void Rev(double motorSpeed){
      ShootingMotor1.set(-1 * motorSpeed);
      ShootingMotor2.set(motorSpeed);
    }
    public void Deliver(double motorSpeed){
      StagingMotor1.set(motorSpeed);
      StagingMotor2.set(-1 * motorSpeed);
    }
    public void ShootOff(){
      ShootingMotor1.set(0);
      ShootingMotor2.set(0);
      StagingMotor1.set(0);
      StagingMotor2.set(0);
    }
    public void StagingIn(){
      StagingMotor1.set(.6);
      StagingMotor2.set(-.6);
    }
    public void StagingOff(){
      StagingMotor1.set(0);
      StagingMotor2.set(0);
    }
    public void ShootOn(){
      ShootingMotor1.set(-.5);
      StagingMotor2.set(.2);
    }
    public void ShootReverse(){
      ShootingMotor1.set(.2);
      ShootingMotor2.set(-.2);
      StagingMotor1.set(.15);
      StagingMotor2.set(-.15);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
