// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DefaultDriveTrain extends CommandBase {
  /** Creates a new DefaultDriveTrain. */
  private DriveTrain driveTrain;
  private XboxController xbox;

  public DefaultDriveTrain(DriveTrain driveTrain, XboxController xbox) {

    this.driveTrain = driveTrain;
    this.xbox = xbox;

    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.dtInit();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //driveTrain.rightSide(xbox.getRawAxis(5) * 0.5);
    //driveTrain.leftSide(xbox.getRawAxis(1) * 0.5);
    //driveTrain.tankDrive(xbox.getRawAxis(1) * 0.8, xbox.getRawAxis(5) * 0.8);
    driveTrain.arcadeDrive(xbox.getRawAxis(1), xbox.getRawAxis(4));
    if(driveTrain.detectCollisions() ) {
      xbox.setRumble(RumbleType.kBothRumble, .5);
    } else if(!driveTrain.detectCollisions()) {
      xbox.setRumble(RumbleType.kBothRumble, 0);
    }
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
