// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for controlling the robot with two joysticks, and 2 operator button panels. */
public class FullOperatorConsoleOI extends DualJoysticksOI {
  private final CommandJoystick operatorPanelOne;
  private final CommandJoystick operatorPanelTwo;

  private final Trigger[] operatorPanelButtonsOne;
  private final Trigger[] operatorPanelButtonsTwo;

  public FullOperatorConsoleOI(
      int translatePort, int rotatePort, int operatorPanelPortOne, int operatorPanelPortTwo) {
    super(translatePort, rotatePort);
    operatorPanelOne = new CommandJoystick(operatorPanelPortOne);
    operatorPanelTwo = new CommandJoystick(operatorPanelPortTwo);

    this.operatorPanelButtonsOne = new Trigger[13];
    for (int i = 1; i < operatorPanelButtonsOne.length; i++) {
      operatorPanelButtonsOne[i] = operatorPanelOne.button(i);
    }

    this.operatorPanelButtonsTwo = new Trigger[13];
    for (int i = 1; i < operatorPanelButtonsTwo.length; i++) {
      operatorPanelButtonsTwo[i] = operatorPanelTwo.button(i);
    }
  }

  // Operator Panel
  /*
  public Trigger operatorSpeakerButton() {
    return operatorPanelButtonsTwo[10];
  }

  public Trigger operatorAmpButton() {
    return operatorPanelButtonsTwo[3];
  }

  public Trigger operatorFeedButton() {
    return operatorPanelButtonsTwo[1];
  }

  public Trigger operatorEjectButton() {
    return operatorPanelButtonsTwo[2];
  }

  public Trigger slowModeSwitch() {
    return operatorPanelButtonsOne[3];
  }

  public Trigger armClimberSwitch() {
    return operatorPanelButtonsOne[9];
  }

  public Trigger autoClimbButton() {
    return operatorPanelButtonsOne[4];
  }

  public Trigger operatorObjectDetectionAssistButton() {
    return operatorPanelButtonsTwo[12];
  }

  public Trigger sourceLoadButton() {
    return operatorPanelButtonsOne[1];
  }

  public Trigger alignToClimbButton() {
    return operatorPanelButtonsTwo[4];
  }

  public Trigger popShotToggleButton() {
    return operatorPanelButtonsTwo[11];
  }*/
}
