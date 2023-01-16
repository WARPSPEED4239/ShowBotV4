package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CannonRevolve;

public class CannonRevolveSpin extends CommandBase {
  private final CannonRevolve mCannonRevolve;
  private final int mTargetNumberOfBarrels;
  private final double mPercentOutput;
  private int mNumberOfBarrelsAdvanced;
  private boolean waitingForLimit;

  public CannonRevolveSpin(CannonRevolve cannonRevolve, int targetNumberOfBarrels, double percentOutput) {
    mCannonRevolve = cannonRevolve;
    mTargetNumberOfBarrels = targetNumberOfBarrels;
    mPercentOutput = percentOutput;
    addRequirements(mCannonRevolve);
  }

  @Override
  public void initialize() {
    mNumberOfBarrelsAdvanced = 0;
    
    if (mCannonRevolve.getRevolveLimitSwitch()) {
      waitingForLimit = false;
    } else {
      waitingForLimit = true;
    }
  }

  @Override
  public void execute() {
    mCannonRevolve.setPercentOutput(mPercentOutput);

    if (waitingForLimit) {
      if (mCannonRevolve.getRevolveLimitSwitch()) {
        mNumberOfBarrelsAdvanced++;
        waitingForLimit = false;
      }
    } else {
      if (!mCannonRevolve.getRevolveLimitSwitch()) {
        waitingForLimit = true;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    mCannonRevolve.setPercentOutput(0.0);
  }

  @Override
  public boolean isFinished() {
    if (mNumberOfBarrelsAdvanced == mTargetNumberOfBarrels) {
      return true;
    } else {
      return false;
    }
  }
}
