// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;

public class SwerveModule extends SubsystemBase {
  private static final double kWheelRadius = 0.0508; // In meters
  // TODO: Find out the resolution
  private static final int kEncoderResolution = 4096;

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration =
    8 * Math.PI; // Radians per second per second

  private final CANSparkMax m_driveMotor;
  private final TalonSRX m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final Encoder m_steerEncoder;

    // TODO: Tune all below
  private final PIDController m_drivePIDController = new PIDController(2, 4, 0.1);

  private final ProfiledPIDController m_turningPIDController =
    new ProfiledPIDController(
            2,
            0.1,
            0.1,
            new TrapezoidProfile.Constraints(
                kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.1, 0.1);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0.1, 0.1);
    
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int steerEncoderOne,
      int steerEncoderTwo) {

    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new TalonSRX(turningMotorChannel);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_steerEncoder = new Encoder(steerEncoderOne, steerEncoderTwo);

    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveRate(), new Rotation2d(getRotationPosition()));
    }

  public void setDesiredState(SwerveModuleState desiredState) {

    if (Math.abs(desiredState.speedMetersPerSecond) < 0.01){
      stop();
      return;
    }

    SwerveModuleState state = 
      SwerveModuleState.optimize(desiredState, new Rotation2d(getRotationPosition()));

    final double driveOutput =
      m_drivePIDController.calculate(getDriveRate(), state.speedMetersPerSecond);

    final double turnOutput =
      m_turningPIDController.calculate(getRotationPosition(), state.angle.getRadians());
        
    final double turnFeedforward = 
      m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(driveOutput);
    m_turningMotor.set(ControlMode.PercentOutput, turnOutput + turnFeedforward);
  }

  public void stop() {
    m_driveMotor.set(0);
    m_turningMotor.set(ControlMode.PercentOutput, 0);
  }

  public double getDriveRate() {
    // Meters per second
    return(m_driveEncoder.getVelocity() / 6.67 * (2 * Math.PI * kWheelRadius) / 60);
  }

    public double getRotationPosition() {
      // Radians
    if(Constants.ABSOLUTE_ENCODER) {
      return((m_turningMotor.getSelectedSensorPosition() / kEncoderResolution) * (2 * Math.PI));
    } else {
      return((m_steerEncoder.get() / 414.16666667) * (2 * Math.PI));
    }
  }

  public void stop() {
    m_driveMotor.set(0);
    m_turningMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
