// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class BalancingCommand extends CommandBase {
  /** Creates a new BalancingCommand. */
  private double kpFwd = -0.05;
  private double robotAngle;
  public AHRS navX = RobotContainer.getDrivetrain().getNavx();
  public DrivetrainSubsystem driving = RobotContainer.getDrivetrain();

  public BalancingCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.getDrivetrain());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    robotAngle = navX.getPitch();
    SmartDashboard.putNumber("Pitch",robotAngle);
    if(robotAngle < .25 && robotAngle > -.25){
    driving.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }else{
    driving.drive(new ChassisSpeeds(robotAngle * kpFwd, 0.0, 0.0));
    }

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
