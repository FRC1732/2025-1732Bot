// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rgb;

import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.joint.Joint;
import frc.robot.subsystems.joint.JointConstants;

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

  private Joint joint;

  private SpecialMode specialMode = SpecialMode.NONE;

  public StatusRgb(Joint joint) {
    timer = new Timer();
    this.joint = joint;
  }

  public void acquiredCoral() {
    timer.start();
    targetElapsedTimeSeconds = 1.5;
    specialMode = SpecialMode.CORAL_CAPTURED;
    System.out.println("Started coral special");
  }

  public void setScoringLevel(ScoringLevel scoringLevel) {
    this.scoringLevel = scoringLevel;
  }

  public void setScoringPosition(ScoringPosition scoringPosition) {
    this.scoringPosition = scoringPosition;
  }

  private void updateButtonState() {
    String build = "";

    build += "L" + scoringLevel.getLevel();
    
    int position = scoringPosition.getPosition();
    build += "P" + (position >= 10 ? "" : "0" ) +  position;

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

    } else if (joint.getSetPoint() == JointConstants.JOINT_CORAL_SETPOINT) {
      setMode(6);

    } else if (joint.getSetPoint() == JointConstants.JOINT_LV1_SETPOINT) {
      setMode(2);

    } else if (joint.getSetPoint() == JointConstants.JOINT_LV2_SETPOINT) {
      setMode(3);

    } else if (joint.getSetPoint() == JointConstants.JOINT_LV3_SETPOINT) {
      setMode(4);

    } else if (joint.getSetPoint() == JointConstants.JOINT_LV4_SETPOINT) {
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
