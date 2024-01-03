// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TestSparkMotor extends CommandBase {
  
  DriveTrain driveTrain;
  int motorID;
  
  /** Creates a new TestSparkMotor. */
  public TestSparkMotor(DriveTrain driveTrain, int motorID) {
    this.driveTrain = driveTrain;
    this.motorID = motorID;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.stopMotors(); // ensure all motors are stopped before testing
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.setSpecificMotorSpeed(motorID, .65);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
