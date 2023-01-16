package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CannonAngleAdjust;

public class CannonAimSetPercentOutputWithController extends CommandBase {
  private final CannonAngleAdjust mCannonAngleAdjust;
  private final CommandXboxController mXbox;

  public CannonAimSetPercentOutputWithController(CannonAngleAdjust cannonAngleAdjust, CommandXboxController xbox) {
    mCannonAngleAdjust = cannonAngleAdjust;
    mXbox = xbox;
    addRequirements(mCannonAngleAdjust);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double mOutput = mXbox.getRawAxis(5);

    if (mOutput < 0.05 && mOutput > -0.05) {
      mCannonAngleAdjust.setPercentOutput(0.0);
    } else {
      mCannonAngleAdjust.setPercentOutput(mOutput);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
