package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class BalancingCommandPID extends CommandBase {
  /** Creates a new BalancingCommand. */
  private double kpFwd = -0.05;
  private double robotAngle;
  public AHRS navX = RobotContainer.getDrivetrain().getNavx();
  public DrivetrainSubsystem driving = RobotContainer.getDrivetrain();
  PIDController anglePid = new PIDController(0.05, 0.01, 0);

  public BalancingCommandPID() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.getDrivetrain());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Pitch",robotAngle);
    robotAngle = navX.getPitch();
    //driving.drive(new ChassisSpeeds(anglePid.calculate(robotAngle, 0), 0, 0));
    if(robotAngle < .9 && robotAngle > -.9){
    
    }else{
      driving.drive(new ChassisSpeeds(anglePid.calculate(robotAngle, 0), 0, 0));
    }
    /*if(robotAngle < .25 && robotAngle > -.25){
    driving.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }else{
    driving.drive(new ChassisSpeeds(robotAngle * kpFwd, 0.0, 0.0));
    }*/

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

