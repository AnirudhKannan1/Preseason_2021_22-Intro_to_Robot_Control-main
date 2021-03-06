// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveForward extends CommandBase {

  private final DriveTrain _driveTrain;
  
  private final double _Distance = 0;
  private final double _Speed = 0;

  /** Creates a new TankDrive. */
  public DriveForward(DriveTrain dt, double distance, double speed) {
    // Use uirements() here to declare subsystem dependencies.
    _driveTrain = dt;

    distance = _Distance;
    speed = _Speed;


    addRequirements(_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    _driveTrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
      _driveTrain.tankDrive(_Speed, _Speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    System.out.println(_driveTrain.getPosition());
      _driveTrain.tankDrive(0, 0);
      _driveTrain.resetEncoders();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
      return _driveTrain.getPosition() > _Distance;
  }
}
