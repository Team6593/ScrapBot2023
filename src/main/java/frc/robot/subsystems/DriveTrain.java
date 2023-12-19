// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.hyperdrive.hyperlib.Vector2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathRamsete;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  private Spark masterRight = new Spark(1);
  private Spark masterLeft = new Spark(2);
  private Spark followerRight = new Spark(3);
  private Spark followerLeft = new Spark(4);

  private MotorControllerGroup right = new MotorControllerGroup(masterRight, followerRight);
  private MotorControllerGroup left = new MotorControllerGroup(masterLeft, followerLeft);

  // horrible code, trying to make PlathPlanner work, I'm not using CAN motors or any encoders
  // so I'm going off of the NavX.
  private DifferentialDrive drive = new DifferentialDrive(left, right);
  private NavX gyro = new NavX();
  Rotation2d zero = new Rotation2d();
  public Pose2d convertToPose(Vector2 vector2) {
    // Assuming you want a default rotation of 0.0 for simplicity

    // Create a new Pose2d object using the x and y coordinates from Vector2
    return new Pose2d(vector2.x, vector2.y, zero);
  }

  Pose2d robotPose;

  public Pose2d resetPosition() {
    gyro.reset();
    Pose2d origin = new Pose2d(0, 0,zero);
    return origin;
  }

  Pose2d reset = resetPosition();

  Consumer<Pose2d> resetPose = pose -> resetPosition();
  ChassisSpeeds currentRobotSpeed;

  Consumer<ChassisSpeeds> output = speeds -> {
    // Assuming that your DifferentialDrive object is named 'drive'
    drive.arcadeDrive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
  };

  @Override
  public void periodic() {
    currentRobotSpeed = gyro.getRobotSpeed();
    robotPose = convertToPose(gyro.getRobotPosition());
  }

  Supplier<Pose2d> robotPoseSupplier = () -> robotPose;
  Supplier<ChassisSpeeds> speedsSupplier = () -> currentRobotSpeed;

  // basically, I don't care about replanning or any error correction type of stuff.
  ReplanningConfig replanningConfig = new ReplanningConfig(false, false, 1.0, .5); 
  public DriveTrain() {
    
    AutoBuilder.configureRamsete(robotPoseSupplier, resetPose, speedsSupplier, output, replanningConfig, this);
    
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

  // collision detection code
  double lastWorldLinearAccelerationX;
  double lastWorldLinearAccelerationY;
  double collisionThreshold = 0.5f;

  public boolean detectCollisions() {
    boolean collisionDetected = false;
    double currentWorldLinearAccelerationX = gyro.navX.getWorldLinearAccelX();
    double currentJerkX = currentWorldLinearAccelerationX - lastWorldLinearAccelerationX;
    lastWorldLinearAccelerationX = currentWorldLinearAccelerationX;
    double currentWorldLinearAccelerationY = gyro.navX.getWorldLinearAccelY();
    double currentJerkY = currentWorldLinearAccelerationY - lastWorldLinearAccelerationY;
    lastWorldLinearAccelerationY = currentWorldLinearAccelerationY;

    // where all the math happen
    if ( ( Math.abs(currentJerkX) > collisionThreshold ) ||
      ( Math.abs(currentJerkY) > collisionThreshold) ) {
      collisionDetected = true;
      System.out.println("COLLISION DETECTED");
    }

    return collisionDetected;
  }


  public void displayNavXData(){}
}
