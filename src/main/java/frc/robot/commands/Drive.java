// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Drive extends CommandBase {

  Drivetrain m_drivetrain;

  DoubleSupplier m_xMove;
  DoubleSupplier m_yMove;
  DoubleSupplier m_rot;

  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(Drivetrain.kMaxSpeed);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(Drivetrain.kMaxSpeed);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(Drivetrain.kMaxAngularSpeed);

  /** Creates a new Drive. */
  public Drive(Drivetrain drivetrain, DoubleSupplier xMove, DoubleSupplier yMove, DoubleSupplier rot) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    m_drivetrain = drivetrain;

    m_xMove = xMove;
    m_yMove = yMove;
    m_rot = rot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final var xSpeed = 
      -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_yMove.getAsDouble(), 0.02)) 
        * Drivetrain.kMaxSpeed;

    final var ySpeed = 
      -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_xMove.getAsDouble(), 0.02)) 
        * Drivetrain.kMaxSpeed;

    final var rot = 
      -m_rotLimiter.calculate(MathUtil.applyDeadband(m_rot.getAsDouble(), 0.02)) 
        * Drivetrain.kMaxAngularSpeed;
    
    
    m_drivetrain.drive(xSpeed, ySpeed, rot, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
