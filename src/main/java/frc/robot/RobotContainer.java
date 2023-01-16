package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CannonAimSetPercentOutputWithController;
import frc.robot.commands.CannonFiringSolenoidSetState;
import frc.robot.commands.CannonLoadingSolenoidSetState;
import frc.robot.commands.CannonRevolveSetPercentOutput;
import frc.robot.commands.CannonRevolveSpin;
import frc.robot.commands.DrivetrainArcadeDrive;
import frc.robot.subsystems.Cannon;
import frc.robot.subsystems.CannonAngleAdjust;
import frc.robot.subsystems.CannonRevolve;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
  private XboxController mXbox = new XboxController(0);
  
  private final Cannon mCannon = new Cannon();
  private final CannonRevolve mCannonRevolve = new CannonRevolve();
  private final CannonAngleAdjust mCannonAngleAdjust = new CannonAngleAdjust();
  private final Drivetrain mDrivetrain = new Drivetrain();

  public RobotContainer() {
    mCannon.setDefaultCommand(cannonReloading());
    mCannonRevolve.setDefaultCommand(new CannonRevolveSetPercentOutput(mCannonRevolve, 0.0));
    mCannonAngleAdjust.setDefaultCommand(new CannonAimSetPercentOutputWithController(mCannonAngleAdjust, mXbox));
    mDrivetrain.setDefaultCommand(new DrivetrainArcadeDrive(mDrivetrain, mXbox));

    configureButtonBindings();

    UsbCamera cam0 = CameraServer.startAutomaticCapture(0);
		cam0.setResolution(320, 240);
    cam0.setFPS(10);
  }

  private void configureButtonBindings() {
    JoystickButton xButtonA, xButtonB, xButtonX, xButtonY, xButtonLeftBumper, xButtonRightBumper, xButtonLeftStick,
        xButtonRightStick;
        
    xButtonA = new JoystickButton(mXbox, 1);
		xButtonB = new JoystickButton(mXbox, 2);
		xButtonX = new JoystickButton(mXbox, 3);
		xButtonY = new JoystickButton(mXbox, 4);
		xButtonLeftBumper = new JoystickButton(mXbox, 5);
		xButtonRightBumper = new JoystickButton(mXbox, 6);
		xButtonLeftStick = new JoystickButton(mXbox, 9);
    xButtonRightStick = new JoystickButton(mXbox, 10);

    xButtonA.whenPressed(new ConditionalCommand(
      new ConditionalCommand(
        cannonFire(), 
        new CannonRevolveSpin(mCannonRevolve, 1, -Constants.CANNON_ROTATION_SPEED), 
        () -> mCannonRevolve.getRevolveLimitSwitch()), 
      new InstantCommand(), 
      () -> mCannon.getFiringTankPressure() >= Constants.MIN_FIRING_PRESSURE)
      );
      
    xButtonB.whenPressed(new CannonRevolveSpin(mCannonRevolve, 8, 1.0));
    xButtonX.whenPressed(new CannonRevolveSpin(mCannonRevolve, 8, -1.0));

    xButtonLeftBumper.whenPressed(new CannonRevolveSpin(mCannonRevolve, 1, -Constants.CANNON_ROTATION_SPEED));
    xButtonRightBumper.whenPressed(new CannonRevolveSpin(mCannonRevolve, 1, Constants.CANNON_ROTATION_SPEED));
  }

  /**
   * This command should be the default command for the Cannon. The Cannon Subsytem consists of 
   * the firing and loading solenoids and the firing tank pressure sensor.
   * 
   * This command should be contiunously being called, unless cannonFire() is called. When being 
   * called, the following should happen:
   * 1) Check if the firing tank pressure is less than or equall to minimum firing pressure
   *    Known: Need to close firing solenoid and open loading solenoid to fill firing tank for next shot 
   *    a) Set firing solenoid to closed
   *    b) Set loading solenoid to open
   * 
   * 2) Check if the firing tank presure is greater than the maximum firing pressure
   *    Known: Need to close the loading solenoid as we do not need more air populating into the firing tank
   *    a) Keep the firing solenoid state unchanged
   *    b) Set loading solenoid to closed
   * 
   * 3) Else hold the Cannon's current state
   *    Known: Let the firing tanks pressure be X, this case will run iff min_pressure < X < max_pressure
   *    a) Hold Cannon's current state by running an instant command
   * 
   * Known issues:
   * A) A default command cannot end, which all of the below commands do
   * B) The if statement in this command is only checked once on RobotInit and never again, not 
   * allowing the commands being called to be updated ever
   * 
   * @return The command that should be ran
   */
  public Command cannonReloading() {
    if (mCannon.getFiringTankPressure() <= Constants.MIN_FIRING_PRESSURE) {
      return new SequentialCommandGroup(
        new CannonFiringSolenoidSetState(mCannon, false).withTimeout(0.5),
        new CannonLoadingSolenoidSetState(mCannon, true).withTimeout(0.5)
      );
    } else if (mCannon.getFiringTankPressure() >= Constants.MAX_FIRING_PRESSURE) {
      return new CannonLoadingSolenoidSetState(mCannon, false).withTimeout(0.5);
    } else {
      return new InstantCommand();
    }
  }

  public Command cannonFire() {
    return new SequentialCommandGroup(
      new CannonLoadingSolenoidSetState(mCannon, false).withTimeout(1.0),
      new CannonFiringSolenoidSetState(mCannon, true).withTimeout(0.5),
      new CannonRevolveSpin(mCannonRevolve, 1, Constants.CANNON_ROTATION_SPEED)
    );
  }
}
