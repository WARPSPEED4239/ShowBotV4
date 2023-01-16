package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;

public class DrivetrainArcadeDrive extends CommandBase {
  private final Drivetrain mDrivetrain;
  private final CommandXboxController mXbox;

  public DrivetrainArcadeDrive(Drivetrain drivetrain, CommandXboxController xbox) {
    mDrivetrain = drivetrain;
    mXbox = xbox;
    addRequirements(mDrivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double move = mXbox.getRightTriggerAxis() - mXbox.getLeftTriggerAxis();
    double rotate = (.533333 * Math.pow(mXbox.getLeftX(), 3) + .466666 *  mXbox.getLeftX());

    if (rotate > 0.85){
      rotate = 0.85;
    }
    else if (rotate < -0.85) {
      rotate = -0.85;
    }
    
    //for anyone who sees this- I have no idea why I had to flip these it just stopped working and it fixed it somehow. I still don't understand what the problem is >:(
    mDrivetrain.arcadeDrive(rotate, move);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
