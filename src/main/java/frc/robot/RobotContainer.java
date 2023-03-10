package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.BalancingCommand;
import frc.robot.commands.BalancingCommandPID;
import frc.robot.commands.ChangeSpeed;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RobotContainer {
//Creates an instance of the DrivetrainSubsystem class. 
  private static final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

//Joystick - Creates joystick objects using the ID number from constants and the Joystick class 
 public final Joystick leftJoy = new Joystick(Constants.leftJoystick);
 public final static Joystick rightJoy = new Joystick(Constants.rightJoystick);
 public final Joystick gamepad = new Joystick(Constants.gamepad);

 //Joystick button - Declares the names for each of the joystick buttons 
   public JoystickButton rTrigger;
   public JoystickButton lTrigger;
   public JoystickButton lInside;
   public JoystickButton rInside;
   public JoystickButton lOutside;
   public JoystickButton rOutside;
   public JoystickButton rBottom;
   public JoystickButton lBottom;

//GamePad - Declares the names for each of the gamepad buttons
 public JoystickButton gamepadX;	
 public JoystickButton gamepadA;
 public JoystickButton gamepadY;
 public JoystickButton gamepadB;
 public JoystickButton gamepadStart;
 public JoystickButton gamepadSelect;
 public JoystickButton gamepadL1;
 public JoystickButton gamepadR1;
 public JoystickButton gamepadR3;
 public JoystickButton gamepadL3;

 //This is the RobotContainer method 
  public RobotContainer() {
  //Sets default command for the drivetrainSubsystem. A default command will run when nothing else is so it is perfect for a teleop driving command
    //Runs the DefaultDriveCommand with the 4 parameters plugged into it. 
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(m_drivetrainSubsystem,
        () -> -modifyAxis(rightJoy.getY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND / Constants.METERS_PER_SECOND_DIVIDED,
        () -> -modifyAxis(rightJoy.getX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND / Constants.METERS_PER_SECOND_DIVIDED,
        () -> -modifyAxis(rightJoy.getZ()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / (Constants.METERS_PER_SECOND_DIVIDED * 2)));

  //Runs the configureButtonBindings function created below. 
    configureButtonBindings();
  }

//Defines the function configureButtonBindings
  private void configureButtonBindings() {

//Creates button objects for each button in a similar manner as the joysticks from above
    // Joystick
    rTrigger = new JoystickButton(rightJoy, Constants.JoystickTriggerR);
    lTrigger = new JoystickButton(leftJoy, Constants.JoystickTriggerL);
    rInside = new JoystickButton(rightJoy, Constants.JoystickRightInside);
    lInside = new JoystickButton(leftJoy, Constants.JoystickLeftInside);
    rOutside = new JoystickButton(rightJoy, Constants.JoystickRightOutside);
    lOutside = new JoystickButton(leftJoy, Constants.JoystickLeftOutside);
    rBottom = new JoystickButton(rightJoy, Constants.JoystickRightBottom);
    lBottom = new JoystickButton(leftJoy, Constants.JoystickLeftBottom);

//Same thing as joystick but for the gamepad buttons
    // Gamepad
    gamepadX = new JoystickButton(gamepad, Constants.GamepadX);
		gamepadA = new JoystickButton(gamepad, Constants.GamepadA);
		gamepadY = new JoystickButton(gamepad, Constants.GamepadY);
		gamepadB = new JoystickButton(gamepad, Constants.GamepadB);
		gamepadR1 = new JoystickButton(gamepad, Constants.GamepadR1);
    gamepadL1 = new JoystickButton(gamepad, Constants.GamepadL1);
    gamepadR3 = new JoystickButton(gamepad, Constants.GamepadR3);
    gamepadL3 = new JoystickButton(gamepad, Constants.GamepadL3);
    gamepadSelect = new JoystickButton(gamepad, Constants.GamepadSelect);
   
 
//This is all of the commands called by the joystick buttons - I like to seperate gamepad and joystick for easily finding a command
    //Joystick Functions
    rBottom.whenPressed(m_drivetrainSubsystem::zeroGyroscope); //This syntax creates an 'instant command' \
    lBottom.whenPressed(m_drivetrainSubsystem::zeroPitch);

    
//Not much to say about this one, it is just the gamepad version of the joysticks above 
    //Gamepad Functions
    // Assign Commands To Buttons
    //rTrigger.whenHeld(new BalancingCommand());
    rTrigger.whenHeld(new BalancingCommandPID());
    lTrigger.whenPressed(new ChangeSpeed());
    
  }

//Now to the fun stuff. This is the command that will run during autonomous period. To change command, just replace TwoBallAuto() with your command of choice. 
  public Command getAutonomousCommand() {
    return null;
  }

//This function returns the name of the drivetrainSubsystem so that it can be used by other commands and subsystems. 
  public static DrivetrainSubsystem getDrivetrain(){
    return m_drivetrainSubsystem;
  }
//Defines the deadband function. A deadband doesn't allow values under a certain value to get passed to the motors to avoid them moving at the slightest bump of a joystick.
  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

//This is a function that uses both the deadband function and squares the value. Squaring allows more accuracy at smaller values while scaling up to higher values when joystick is pressed farther.
/**
 * This command applys a deadband and squares the axis. Deadband ignores very small value changes so motors do not get burned up from constantly moving .00001. Squaring the axis
 * turns the linear rate of change into quadratic. Pretty much creates a parabola curve so small values are easier to drive with for precison and larger values will change faster allowing faster moving.
 * 
 * @param Value: Put the getAxis command in here.
 * @return Returns the modified axis 
 */
  public static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
