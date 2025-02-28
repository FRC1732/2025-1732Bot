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

  // Robot Movement

  @Override
  public double getTranslateX() {
    double input = translateJoystick.getY();

    return Math.copySign(Math.pow(input, 2), input);
  }

  @Override
  public double getTranslateY() {
    double input = translateJoystick.getX();

    return Math.copySign(Math.pow(input, 2), input);
  }

  @Override
  public double getRotate() {
    double input = rotateJoystick.getX();

    return Math.copySign(Math.pow(input, 3), input);
  }

  @Override
  public Trigger hybridIntakeCoralButton() {
    return translateJoystickButtons[5];
  }

  // Translation Joystick Buttons

  @Override
  public Trigger intakeCoralButton() {
    return translateJoystickButtons[1];
  }

  @Override
  public Trigger aimAtNetButton() {
    return translateJoystickButtons[3];
  }

  @Override
  public Trigger intakeAlgaeButton() {
    return translateJoystickButtons[2];
  }

  // @Override
  // public Trigger operatorExtendClimber() {
  //   return translateJoystickButtons[5];
  // }

  // @Override
  // public Trigger operatorRetractClimber() {
  //   return translateJoystickButtons[4];
  // }

  // Rotate Joystick Buttons

  @Override
  public Trigger scoreCoralButton() {
    return rotateJoystickButtons[1];
  }

  @Override
  public Trigger ejectCoralButton() {
    return rotateJoystickButtons[2];
  }

  @Override
  public Trigger ejectAlgaeButton() {
    return rotateJoystickButtons[3];
  }

  @Override
  public Trigger pluckAlgaeButton() {
    return rotateJoystickButtons[4];
  }

  @Override
  public Trigger resetGyroButton() {
    return rotateJoystickButtons[8];
  }

  // SysId Buttons @todo disable these after using sysid
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
