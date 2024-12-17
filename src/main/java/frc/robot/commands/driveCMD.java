// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;

public class driveCMD extends Command {
  Drivetrain drivetrain;
  CommandXboxController operatorController;

  /** Creates a new driveCMD. */
  public driveCMD(Drivetrain drivetrain, CommandXboxController operatorController) {
    this.drivetrain = drivetrain;
    this.operatorController = operatorController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(
      MathUtil.applyDeadband(operatorController.getLeftX(), 0.25),
      MathUtil.applyDeadband(operatorController.getLeftY(), 0.25),
      MathUtil.applyDeadband(operatorController.getRightX(), 0) * Math.PI,
      true);
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
