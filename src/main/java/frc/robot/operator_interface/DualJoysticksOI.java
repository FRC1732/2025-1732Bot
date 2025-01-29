// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for controlling the robot with two joysticks. */
public class DualJoysticksOI implements OperatorInterface {
  private final CommandJoystick translateJoystick;
  private final CommandJoystick rotateJoystick;
  private final Trigger[] translateJoystickButtons;
  private final Trigger[] rotateJoystickButtons;

  public DualJoysticksOI(int translatePort, int rotatePort) {
    translateJoystick = new CommandJoystick(translatePort);
    rotateJoystick = new CommandJoystick(rotatePort);

    // buttons use 1-based indexing such that the index matches the button number; leave index 0 set
    // to null
    this.translateJoystickButtons = new Trigger[13];
    this.rotateJoystickButtons = new Trigger[13];

    for (int i = 1; i < translateJoystickButtons.length; i++) {
      translateJoystickButtons[i] = translateJoystick.button(i);
      rotateJoystickButtons[i] = rotateJoystick.button(i);
    }
  }

  @Override
  public double getTranslateX() {
    double input = -translateJoystick.getY();

    return Math.copySign(Math.pow(input, 2), input);
  }

  @Override
  public double getTranslateY() {
    double input = -translateJoystick.getX();

    return Math.copySign(Math.pow(input, 2), input);
  }

  @Override
  public double getRotate() {
    double input = rotateJoystick.getX();

    return Math.copySign(Math.pow(input, 2), input);
  }

  @Override
  public Trigger getFieldRelativeButton() {
    return translateJoystickButtons[3];
  }

  @Override
  public Trigger getResetGyroButton() {
    return rotateJoystickButtons[8];
  }

  @Override
  public Trigger getLock180Button() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger getXStanceButton() {
    return rotateJoystickButtons[4];
  }

  @Override
  public Trigger getVisionIsEnabledSwitch() {
    // vision is always enabled with dual joysticks as there is no switch to disable
    return new Trigger(() -> true);
  }

  @Override
  public Trigger getResetPoseToVisionButton() {
    return translateJoystickButtons[10];
  }

  /*@Override
    public Trigger getArmTriggerForward() {
      return rotateJoystickButtons[3];
    }
  */

  /*@Override
    public Trigger getArmTriggerBackwards() {
      return rotateJoystickButtons[2];
    }
  */

  @Override
  public Trigger getClawTriggerForwards() {
    return translateJoystickButtons[4];
  }

  @Override
  public Trigger getClawTriggerBackwards() {
    return rotateJoystickButtons[1];
  }

  @Override
  public Trigger getIntakeCoral() {
    return translateJoystickButtons[1];
  }

  @Override
  public Trigger scoreL1() {
    return rotateJoystickButtons[4];
  }

  @Override
  public Trigger scoreL2() {
    return rotateJoystickButtons[3];
  }

  @Override
  public Trigger scoreL3() {
    return rotateJoystickButtons[5];
  }

  @Override
  public Trigger scoreL4() {
    return rotateJoystickButtons[2];
  }

  // @todo disable these after using sysid
  @Override
  public Trigger getSysIdDynamicForward() {
    return rotateJoystickButtons[6];
  }

  @Override
  public Trigger getSysIdDynamicReverse() {
    return rotateJoystickButtons[7];
  }

  @Override
  public Trigger getSysIdQuasistaticForward() {
    return rotateJoystickButtons[11];
  }

  @Override
  public Trigger getSysIdQuasistaticReverse() {
    return rotateJoystickButtons[10];
  }
}
