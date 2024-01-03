// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveForward extends CommandBase {
  
  DriveTrain driveTrain;

  /** Creates a new DriveForward. */
  public DriveForward(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.dtInit();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.moveToPosition(.2);
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
