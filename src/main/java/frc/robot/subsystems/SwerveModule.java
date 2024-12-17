// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {

  CANSparkFlex driveMotor;
  CANSparkMax turnMotor;

  RelativeEncoder driveEncoder;
  AbsoluteEncoder turnEncoder;

  SparkPIDController drivePIDController;
  SparkPIDController turnPIDController;

  double chassisAngularOffset = 0;
  SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorID, int turnMotorID, double chassisAngularOffset) {

    driveMotor = new CANSparkFlex(driveMotorID, MotorType.kBrushless);
    turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getAbsoluteEncoder();

    drivePIDController = driveMotor.getPIDController();
    turnPIDController = turnMotor.getPIDController();

    drivePIDController.setFeedbackDevice(driveEncoder);
    turnPIDController.setFeedbackDevice(turnEncoder);

    turnEncoder.setInverted(true);

    driveEncoder.setPositionConversionFactor(ModuleConstants.DRIVE_ENCODER_POS_FACTOR);
    driveEncoder.setVelocityConversionFactor(ModuleConstants.DRIVE_ENCODER_VELOCITY_FACTOR);
    turnEncoder.setPositionConversionFactor(ModuleConstants.TURN_ENCODER_POS_FACTOR);
    turnEncoder.setVelocityConversionFactor(ModuleConstants.TURN_ENCODER_VELOCITY_FACTOR);

    turnPIDController.setPositionPIDWrappingEnabled(true);
    turnPIDController.setPositionPIDWrappingMinInput(ModuleConstants.TURN_ENCODER_POS_MIN_INPUT);
    turnPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.TURN_ENCODER_POS_MAX_INPUT);

    drivePIDController.setP(ModuleConstants.DRIVE_P);
    drivePIDController.setI(ModuleConstants.DRIVE_I);
    drivePIDController.setD(ModuleConstants.DRIVE_D);
    drivePIDController.setFF(ModuleConstants.DRIVE_FF);
    drivePIDController.setOutputRange(ModuleConstants.DRIVE_MIN_OUTPUT, ModuleConstants.DRIVE_MAX_OUTPUT);

    turnPIDController.setP(ModuleConstants.TURN_P);
    turnPIDController.setI(ModuleConstants.TURN_I);
    turnPIDController.setD(ModuleConstants.TURN_D);
    turnPIDController.setFF(ModuleConstants.TURN_FF);
    turnPIDController.setOutputRange(ModuleConstants.TURN_MIN_OUTPUT, ModuleConstants.TURN_MAX_OUTPUT);

    driveMotor.setIdleMode(ModuleConstants.DRIVE_MOTOR_IDLE_MODE);
    turnMotor.setIdleMode(ModuleConstants.TURN_MOTOR_IDLE_MODE);
    driveMotor.setSmartCurrentLimit(ModuleConstants.DRIVE_MOTOR_CURRENT_LIMIT);
    turnMotor.setSmartCurrentLimit(ModuleConstants.TURN_MOTOR_CURRENT_LIMIT);

    driveMotor.burnFlash();
    turnMotor.burnFlash();

    this.chassisAngularOffset = chassisAngularOffset;
    desiredState.angle = new Rotation2d(turnEncoder.getPosition());
    driveEncoder.setPosition(0);

  }

 public void setDesiredState(SwerveModuleState desired) {
    desired.angle = desired.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));
    desiredState = desired;
    desiredState = SwerveModuleState.optimize(desired, Rotation2d.fromRadians(turnEncoder.getPosition()));
    drivePIDController.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity);
    turnPIDController.setReference(desiredState.angle.getRadians(), ControlType.kPosition);
  }

  public double getDesiredSpeed() {
    return desiredState.speedMetersPerSecond;
  }

  public double getActualSpeed() {
    return driveEncoder.getVelocity();
  }

  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(driveEncoder.getPosition(), Rotation2d.fromRadians(turnEncoder.getPosition() - chassisAngularOffset));
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), Rotation2d.fromRadians(turnEncoder.getPosition() - chassisAngularOffset));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
