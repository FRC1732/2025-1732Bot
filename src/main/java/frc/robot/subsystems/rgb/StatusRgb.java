// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rgb;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.armevator.Armevator;
import frc.robot.subsystems.armevator.ArmevatorPose;

public class StatusRgb extends SubsystemBase {
  private DigitalOutput out0 = new DigitalOutput(0);
  private DigitalOutput out1 = new DigitalOutput(1);
  private DigitalOutput out2 = new DigitalOutput(2);
  private DigitalOutput out3 = new DigitalOutput(3);
  private DigitalOutput out4 = new DigitalOutput(4);

  private Timer timer;
  private double targetElapsedTimeSeconds;

  private ScoringLevel scoringLevel;
  private ScoringPosition scoringPosition;

  private NetworkTableInstance table = NetworkTableInstance.getDefault();
  private NetworkTable nt4Table = table.getTable("rgbOperator");
  private StringPublisher publisher = nt4Table.getStringTopic("rgb").publish();

  private Armevator armevator;

  private SpecialMode specialMode = SpecialMode.NONE;

  public StatusRgb(Armevator armevator) {
    timer = new Timer();
    this.armevator = armevator;

    // Set default values
    scoringLevel = ScoringLevel.NONE;
    scoringPosition = ScoringPosition.NONE;
    updateButtonState();
  }

  public void acquiredCoral() {
    timer.start();
    targetElapsedTimeSeconds = 1.5;
    specialMode = SpecialMode.CORAL_CAPTURED;
    System.out.println("Started coral special");
  }

  public void setScoringLevel(ScoringLevel scoringLevel) {
    this.scoringLevel = scoringLevel;
    updateButtonState();
  }

  public void setScoringPosition(ScoringPosition scoringPosition) {
    this.scoringPosition = scoringPosition;
    updateButtonState();
  }

  private void updateButtonState() {
    String build = "";

    build += "L" + scoringLevel.getLevel();

    int position = scoringPosition.getPosition();
    build += "P" + (position >= 10 ? "" : "0") + position;

    publisher.set(build);
  }

  public void setMode(int modeToSet) {
    if (modeToSet % 2 == 1) {
      out0.set(!true);
    } else {
      out0.set(!false);
    }
    modeToSet /= 2;

    if (modeToSet % 2 == 1) {
      out1.set(!true);
    } else {
      out1.set(!false);
    }
    modeToSet /= 2;

    if (modeToSet % 2 == 1) {
      out2.set(!true);
    } else {
      out2.set(!false);
    }
    modeToSet /= 2;

    if (modeToSet % 2 == 1) {
      out3.set(!true);
    } else {
      out3.set(!false);
    }
    modeToSet /= 2;

    if (modeToSet % 2 == 1) {
      out4.set(!true);
    } else {
      out4.set(!false);
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
            setMode(1);
            return;
          default: // do nothing
            break;
        }
      }
    }
    // add more modes once more parts of the robot are added
    if (DriverStation.isDisabled()) {
      setMode(0);

    } else if (armevator.getPose() == ArmevatorPose.CORAL_HP_LOAD) {
      setMode(6);

    } else if (armevator.getPose() == ArmevatorPose.CORAL_L1_SCORE) {
      setMode(2);

    } else if (armevator.getPose() == ArmevatorPose.CORAL_L2_SCORE) {
      setMode(3);

    } else if (armevator.getPose() == ArmevatorPose.CORAL_L3_SCORE) {
      setMode(4);

    } else if (armevator.getPose() == ArmevatorPose.CORAL_L4_SCORE) {
      setMode(5);
    } else {
      setMode(0);
    }
  }

  public enum SpecialMode {
    CORAL_CAPTURED,
    NONE;
  }
}
