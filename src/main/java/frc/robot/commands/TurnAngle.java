// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TurnAngle extends CommandBase {

  private final DriveTrain _driveTrain;
  
  private final double _Angle = 0;
  private final double _Speed = 0;

  /** Creates a new TankDrive. */
  public TurnAngle(DriveTrain dt, double angle, double speed) {
    // Use uirements() here to declare subsystem dependencies.
    _driveTrain = dt;

    angle = _Angle;
    speed = _Speed;


    addRequirements(_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    _driveTrain.resetNav();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
      _driveTrain.tankDrive(_Speed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
      _driveTrain.tankDrive(0, 0);
      _driveTrain.resetNav();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
      return _driveTrain.getAngle() > _Angle;
  }
}
