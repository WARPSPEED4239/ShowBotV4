package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CannonRevolve extends SubsystemBase {
  private final int CURRENT_LIMIT = 45;
  private final double RAMP_RATE = 0.05;
  private final CANSparkMax revolveMotor = new CANSparkMax(Constants.CANNON_REVOLVE_MOTOR, MotorType.kBrushed);
  private final DigitalInput revolveLimitSwitch = new DigitalInput(Constants.CANNON_REVOLVE_LIMIT_SWITCH);
  
  public CannonRevolve() {
    revolveMotor.restoreFactoryDefaults();
    revolveMotor.setInverted(true);
    revolveMotor.setIdleMode(IdleMode.kBrake);
    revolveMotor.setSmartCurrentLimit(CURRENT_LIMIT);
    revolveMotor.setOpenLoopRampRate(RAMP_RATE);
    revolveMotor.burnFlash();
  }

  @Override
  public void periodic() {
  }

  public void setPercentOutput(double output) {
    if (output > 1.0) {
      output = 1.0;
    } else if (output < -1.0) {
      output = -1.0;
    }

    revolveMotor.set(output);
  }

  public boolean getRevolveLimitSwitch() {
    return !revolveLimitSwitch.get();
  }
}
