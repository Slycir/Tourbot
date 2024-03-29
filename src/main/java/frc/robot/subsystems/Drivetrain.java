// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  public static final double kMaxSpeed = 5;
  public static final double kMaxAngularSpeed  = 4 * Math.PI;

  private final Translation2d m_frontLeftLocation = new Translation2d(Constants.WHEELBASE/2, Constants.TRACK/2);
  private final Translation2d m_frontRightLocation = new Translation2d(Constants.WHEELBASE/2, -Constants.TRACK/2);
  private final Translation2d m_backLeftLocation = new Translation2d(-Constants.WHEELBASE/2, Constants.TRACK/2);
  private final Translation2d m_backRightLocation = new Translation2d(-Constants.WHEELBASE/2, -Constants.TRACK/2);

  private final SwerveModule m_frontLeft = new SwerveModule(1, 5, 4, 5);
  private final SwerveModule m_frontRight = new SwerveModule(2, 6, 6, 7);
  private final SwerveModule m_backLeft = new SwerveModule(3, 7, 0, 1);
  private final SwerveModule m_backRight = new SwerveModule(4, 8, 2, 3);

  private final AHRS ahrs = new AHRS(SPI.Port.kMXP);

  private final SwerveDriveKinematics m_kinematics = 
    new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry = 
    new SwerveDriveOdometry(m_kinematics, ahrs.getRotation2d());
  /** Creates a new Drivetrain. */
  public Drivetrain() {
    ahrs.reset();
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // System.out.println("Driving!");
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
          ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, ahrs.getRotation2d())
          // new ChassisSpeeds(xSpeed, ySpeed, rot)
          );
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        ahrs.getRotation2d(),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState());
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();
  }
}
