// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Class for controlling the robot with two joysticks, and 2 operator button
 * panels.
 */
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

  @Override
  public Trigger operatorEjectAll() {
    return operatorPanelButtonsTwo[9];
  }

  @Override
  public Trigger operatorResetGyroButton() {
    return operatorPanelButtonsOne[12];
  }

  @Override
  public Trigger operatorExtendClimber() {
    return operatorPanelButtonsTwo[7];
  }

  @Override
  public Trigger operatorRetractClimber() {
    return operatorPanelButtonsTwo[3];
  }

  @Override
  public Trigger operatorSlowMode() {
    return operatorPanelButtonsTwo[5];
  }

  @Override
  public Trigger operatorL1() {
    return new Trigger(() -> operatorPanelTwo.getX() > 0.5);
  }

  @Override
  public Trigger operatorL2() {
    return new Trigger(() -> operatorPanelTwo.getX() < -0.5);
  }

  @Override
  public Trigger operatorL3() {
    return new Trigger(() -> operatorPanelTwo.getY() < -0.5);
  }

  @Override
  public Trigger operatorL4() {
    return new Trigger(() -> operatorPanelTwo.getX() > 0.5);
  }

  @Override
  public Trigger operatorF1() {
    return operatorPanelButtonsOne[10];
  }

  @Override
  public Trigger operatorF2() {
    return operatorPanelButtonsTwo[12];
  }

  @Override
  public Trigger operatorFR1() {
    return operatorPanelButtonsTwo[2];
  }

  @Override
  public Trigger operatorFR2() {
    return operatorPanelButtonsTwo[8];
  }

  @Override
  public Trigger operatorFL1() {
    return operatorPanelButtonsOne[1];
  }

  @Override
  public Trigger operatorFL2() {
    return operatorPanelButtonsOne[2];
  }

  @Override
  public Trigger operatorBR1() {
    return operatorPanelButtonsTwo[1];
  }

  @Override
  public Trigger operatorBR2() {
    return operatorPanelButtonsTwo[4];
  }

  @Override
  public Trigger operatorBL1() {
    return operatorPanelButtonsOne[6];
  }

  @Override
  public Trigger operatorBL2() {
    return operatorPanelButtonsOne[7];
  }

  @Override
  public Trigger operatorB1() {
    return operatorPanelButtonsTwo[10];
  }

  @Override
  public Trigger operatorB2() {
    return operatorPanelButtonsTwo[6];
  }

  @Override
  public Trigger operatorCoralSideSwitch() {
    return operatorPanelButtonsTwo[8];
  }

  @Override
  public Trigger operatorAlgaeClearingHeightSwitch() {
    return operatorPanelButtonsOne[3];
  }

  @Override
  public Trigger operatorClearAlgaeButton() {
    return operatorPanelButtonsOne[8];
  }

  @Override
  public Trigger operatorAlgaeTargetSwitch() {
    return operatorPanelButtonsTwo[11];
  }

  @Override
  public Trigger operatorFullAutoPlacementSwitch() {
    return operatorPanelButtonsOne[11];
  }

  @Override
  public Trigger operatorVisionIsEnabledSwitch() {
    return operatorPanelButtonsOne[9];
  }
}
