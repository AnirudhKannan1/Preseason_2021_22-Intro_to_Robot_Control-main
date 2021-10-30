// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.Timer;

public class DriveForwardPID extends CommandBase {

  private final DriveTrain _driveTrain;
  
  private  double _Distance = 0;
  private  double _Speed = 0;

  private double kP = 0;
  private double kI = 0;
  private double kD = 0;

  private double error = 0;
  private double errorSum = 0;

  private double _time;
  private Timer _Timer;

  /** Creates a new TankDrive. */
  public DriveForwardPID(DriveTrain dt, double distance, double speed) {
    // Use uirements() here to declare subsystem dependencies.
    _driveTrain = dt;

    distance = _Distance;
    speed = _Speed;

    _time = 0;
    _Timer = new Timer();

    addRequirements(_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    _Timer.reset();
    _Timer.start();
    _driveTrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double _receivedTime = _Timer.get();
    error = _Distance - _driveTrain.getPosition();
    double lastError = _driveTrain.getPosition() - error;

    kP = error / _Distance;
    kI = (error - lastError) * _receivedTime;
    kD = (error - lastError) / _receivedTime;

    _Speed = kP + kI + kD;

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
      return _driveTrain.getPosition() >= _Distance;
  }
}
