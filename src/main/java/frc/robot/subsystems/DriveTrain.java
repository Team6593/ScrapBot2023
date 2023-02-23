// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  private Spark masterRight = new Spark(1);
  private Spark masterLeft = new Spark(2);
  private Spark followerRight = new Spark(3);
  private Spark followerLeft = new Spark(4);

  private MotorControllerGroup right = new MotorControllerGroup(masterRight, followerRight);
  private MotorControllerGroup left = new MotorControllerGroup(masterLeft, followerLeft);

  private DifferentialDrive drive = new DifferentialDrive(left, right);
  
  public DriveTrain() {
    
  }

  public void dtInit(){
    masterRight.setInverted(false);
    masterLeft.setInverted(true);
    followerRight.setInverted(false);    
    followerLeft.setInverted(true);
  }

  public void rightSide(double speed){
    right.set(speed);
  }

  public void leftSide(double speed){
    left.set(speed);
  }

  public void tankDrive(double rSpeed, double lSpeed){
    drive.tankDrive(lSpeed, rSpeed);
    //right.set(rSpeed);
    //left.set(lSpeed);
  }

  public void arcadeDrive(double xSpd, double zRot){
    drive.arcadeDrive(xSpd, zRot, false);
  }

  public void stopMotors(){
    masterRight.stopMotor();
    masterLeft.stopMotor();
    followerRight.stopMotor();
    followerLeft.stopMotor();
    
  }

  public void displayNavXData(){}

  @Override
  public void periodic() {
   // This method will be called once per scheduler run
  }
}
