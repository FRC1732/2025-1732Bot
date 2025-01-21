// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rgb;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StatusRgb extends SubsystemBase {
  private DigitalOutput out0 = new DigitalOutput(0);
  private DigitalOutput out1 = new DigitalOutput(1);
  private DigitalOutput out2 = new DigitalOutput(2);
  private DigitalOutput out3 = new DigitalOutput(3);
  private DigitalOutput out4 = new DigitalOutput(4);

  private Timer timer;
  private double targetElapsedTimeSeconds;

  private SpecialMode specialMode = SpecialMode.NONE;

  public StatusRgb() {
    timer = new Timer();

  }

  
  public void acquiredCoral() {
    timer.start();
    targetElapsedTimeSeconds = 1.5;
    specialMode = SpecialMode.CORAL_CAPTURED;
    System.out.println("Started coral special");
  }

  public void setMode(int modeToSet) {
    if (modeToSet % 2 == 1) {
      out0.set(!true);
    }
    modeToSet /= 2;

    if (modeToSet % 2 == 1) {
      out1.set(!true);
    }
    modeToSet /= 2;

    if (modeToSet % 2 == 1) {
      out2.set(!true);
    }
    modeToSet /= 2;

    if (modeToSet % 2 == 1) {
      out3.set(!true);
    }
    modeToSet /= 2;

    if (modeToSet % 2 == 1) {
      out4.set(!true);
    }
  }

  
  
  @Override
  public void periodic() {
    if (specialMode != SpecialMode.NONE) {
      if (timer.hasElapsed(targetElapsedTimeSeconds)) {
        specialMode = SpecialMode.NONE;
        timer.stop();
        timer.reset();
      } else {
        switch (specialMode) {
          case CORAL_CAPTURED: // blue and gold
            setMode(1);;
            break;
          default: // do nothing
            break;
        }
      }
    }
    // add more modes once more parts of the robot are added
    if (DriverStation.isDisabled()) {
      setMode(0);
    } else {
      setMode(0);
    }  
  }

  public enum SpecialMode {
    CORAL_CAPTURED,
    NONE;
  }
}
