package frc.robot;

import com.ctre.phoenix.CANifier;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CannonAimSetPercentOutputWithController;
import frc.robot.commands.CannonFiringSolenoidSetState;
import frc.robot.commands.CannonLoadingSolenoidSetState;
import frc.robot.commands.CannonRevolve;
import frc.robot.commands.DrivetrainArcadeDrive;
import frc.robot.commands.RGBSetColor;
import frc.robot.subsystems.Cannon;
import frc.robot.subsystems.CannonAngleAdjust;
import frc.robot.subsystems.Drivetrain;
import frc.robot.tools.RGBController;
import frc.robot.tools.RGBController.Color;

public class RobotContainer {
  private CommandXboxController mXbox = new CommandXboxController(0);
  
  private final Cannon mCannon = new Cannon();
  private final CannonAngleAdjust mCannonAngleAdjust = new CannonAngleAdjust();
  private final Drivetrain mDrivetrain = new Drivetrain();
  private final RGBController mRGBController = new RGBController(new CANifier(Constants.CANIFIER));

  public RobotContainer() {
    mCannon.setDefaultCommand(new ConditionalCommand(cannonReloading(), cannonReadyToFire(), () -> mCannon.getFiringTankPressure() < 80.0));
    mCannonAngleAdjust.setDefaultCommand(new CannonAimSetPercentOutputWithController(mCannonAngleAdjust, mXbox));
    mDrivetrain.setDefaultCommand(new DrivetrainArcadeDrive(mDrivetrain, mXbox));

    configureButtonBindings();

    UsbCamera cam0 = CameraServer.startAutomaticCapture(0);
		cam0.setResolution(320, 240);
    cam0.setFPS(10);
  }

  private void configureButtonBindings() {
    mXbox.a().onTrue(new ConditionalCommand(cannonFire(), new InstantCommand(), () -> mCannon.getFiringTankPressure() >= 80.0));
    mXbox.b().onTrue(new CannonRevolve(mCannon, 8, 1.0));
    mXbox.x().onTrue(new CannonRevolve(mCannon, 8, -1.0));

    mXbox.leftBumper().onTrue(new CannonRevolve(mCannon, 1, -0.7));
    mXbox.rightBumper().onTrue(new CannonRevolve(mCannon, 1, 0.7));
  }

  public Command cannonReloading() {
    Command mCommand = new SequentialCommandGroup(
      new ParallelRaceGroup(
        new RGBSetColor(mRGBController, Color.Black),
        new CannonFiringSolenoidSetState(mCannon, false),
        new WaitCommand(0.5)
      ),
      new CannonLoadingSolenoidSetState(mCannon, true)
    );

    return mCommand;
  }
  
  public Command cannonReadyToFire() {
    Command mCommand = new ParallelCommandGroup(
      new RGBSetColor(mRGBController, Color.Red),
      new CannonLoadingSolenoidSetState(mCannon, false)
    );

    return mCommand;
  }

  public Command cannonFire() {
    Color[] redFlashing = {Color.Red, Color.Black};

    Command mCommand = new SequentialCommandGroup(
      new ParallelRaceGroup(
        new CannonFiringSolenoidSetState(mCannon, false),
        new RGBSetColor(mRGBController, redFlashing, 0.2),
        new WaitCommand(1.0)
      ),
      new ParallelRaceGroup(
        new CannonFiringSolenoidSetState(mCannon, true),
        new RGBSetColor(mRGBController, Color.White),
        new WaitCommand(0.5)
      ),
      new ParallelRaceGroup(
        new RGBSetColor(mRGBController, Color.Black),
        new WaitCommand(0.5)
      ),
      new CannonRevolve(mCannon, 1, 0.7)
    );

    return mCommand;
  }

  public RGBController getRGBController() {
    return mRGBController;
  }
}
