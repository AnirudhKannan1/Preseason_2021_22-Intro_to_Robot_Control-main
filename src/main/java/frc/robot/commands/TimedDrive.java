// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.Timer;

public class TimedDrive extends CommandBase {

  private final DriveTrain _driveTrain;
  private final double _leftSpeed;
  private final double _rightSpeed;
  private final double _time;
  private final Timer _timer;
  private boolean done = false;

  /** Creates a new TankDrive. */
  public TimedDrive(DriveTrain dt) {
    // Use uirements() here to declare subsystem dependencies.
    _driveTrain = dt;
    _leftSpeed = 1;
    _rightSpeed = 1;
    _time = 5;
    _timer = new Timer();

    addRequirements(_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _timer.start();
    _timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double _receivedTime = _timer.get();
    if (_receivedTime <= 4) {
      _driveTrain.tankDrive(-1.0 * _leftSpeed, -1.0 * _rightSpeed);
    } else {
      _driveTrain.tankDrive(0, 0);
      done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
