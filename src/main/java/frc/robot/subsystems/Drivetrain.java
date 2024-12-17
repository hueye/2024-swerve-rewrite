// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GlobalConstants;

public class Drivetrain extends SubsystemBase {

  private final SwerveModule fL = new SwerveModule(DriveConstants.flDriveID, DriveConstants.flTurnID, DriveConstants.flChassisOffset);
  private final SwerveModule fR = new SwerveModule(DriveConstants.frDriveID, DriveConstants.frTurnID, DriveConstants.frChassisOffset);
  private final SwerveModule bL = new SwerveModule(DriveConstants.blDriveID, DriveConstants.blTurnID, DriveConstants.blChassisOffset);
  private final SwerveModule bR = new SwerveModule(DriveConstants.brDriveID, DriveConstants.brTurnID, DriveConstants.brChassisOffset);

  private Pigeon2 pigeon = new Pigeon2(GlobalConstants.pigeonID);
  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(DriveConstants.swerveKinematics, getRotation2d(), getModulePosition());

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    pigeon.reset();

  }

  public void drive(double xSpeed, double ySpeed, double omega, boolean fieldRelative) {
    SwerveModuleState[] states = DriveConstants.swerveKinematics.toSwerveModuleStates(fieldRelative
    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omega, pigeon.getRotation2d())
    : new ChassisSpeeds(xSpeed, ySpeed, omega));
  
  fL.setDesiredState(states[0]);
  fR.setDesiredState(states[1]);
  bL.setDesiredState(states[2]);
  bR.setDesiredState(states[3]);
  }  

  public Rotation2d getRotation2d() {
    return pigeon.getRotation2d();
  }

  public void driveChassisSpeeds(ChassisSpeeds speeds) {
    SwerveModuleState[] states = DriveConstants.swerveKinematics.toSwerveModuleStates(speeds);
    fL.setDesiredState(states[0]);
    fR.setDesiredState(states[1]);
    bL.setDesiredState(states[2]);
    bR.setDesiredState(states[3]);
  }

  private SwerveModulePosition[] getModulePosition() {
    return new SwerveModulePosition[] {
      fL.getModulePosition(),
      fR.getModulePosition(),
      bL.getModulePosition(),
      bR.getModulePosition()
    };
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    odometry.resetPosition(getRotation2d(), getModulePosition(), pose);
  }

  public SwerveModuleState[] moduleStates() {
    return new SwerveModuleState[] {
      fL.getState(),
      fR.getState(),
      bL.getState(),
      bR.getState()
    };
  }

  public ChassisSpeeds getSpeeds() {
    return DriveConstants.swerveKinematics.toChassisSpeeds(moduleStates());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("desired speed", fL.getDesiredSpeed());
    SmartDashboard.putNumber("actual speed", fL.getActualSpeed());
    odometry.update(pigeon.getRotation2d(), getModulePosition());
  }
}
