// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.*;

/** Interface for all driver and operator controls. */
public interface OperatorInterface {
  public default double getTranslateX() {
    return 0.0;
  }

  public default double getTranslateY() {
    return 0.0;
  }

  public default double getRotate() {
    return 0.0;
  }

  public default Trigger resetGyroButton() {
    return new Trigger(() -> false);
  }

  public default Trigger xStanceButton() {
    return new Trigger(() -> false);
  }

  public default Trigger slowModeSwitch() {
    return new Trigger(() -> false);
  }

  public default Trigger getSysIdDynamicForward() {
    return new Trigger(() -> false);
  }

  public default Trigger getSysIdDynamicReverse() {
    return new Trigger(() -> false);
  }

  public default Trigger getSysIdQuasistaticForward() {
    return new Trigger(() -> false);
  }

  public default Trigger getSysIdQuasistaticReverse() {
    return new Trigger(() -> false);
  }

  public default Trigger intakeCoralButton() {
    return new Trigger(() -> false);
  }

  public default Trigger scoreCoralButton() {
    return new Trigger(() -> false);
  }

  public default Trigger clearAlgaeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger aimAtNetButton() {
    return new Trigger(() -> false);
  }

  public default Trigger intakeAlgaeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger ejectAllButton() {
    return new Trigger(() -> false);
  }

  public default Trigger operatorEjectAll() {
    return new Trigger(() -> false);
  }

  public default Trigger operatorVisionIsEnabledSwitch() {
    return new Trigger(() -> false);
  }

  public default Trigger operatorResetGyroButton() {
    return new Trigger(() -> false);
  }

  public default Trigger operatorExtendClimber() {
    return new Trigger(() -> false);
  }

  public default Trigger operatorRetractClimber() {
    return new Trigger(() -> false);
  }

  public default Trigger operatorSlowMode() {
    return new Trigger(() -> false);
  }

  public default Trigger operatorL1() {
    return new Trigger(() -> false);
  }

  public default Trigger operatorL2() {
    return new Trigger(() -> false);
  }

  public default Trigger operatorL3() {
    return new Trigger(() -> false);
  }

  public default Trigger operatorL4() {
    return new Trigger(() -> false);
  }

  public default Trigger operatorF1() {
    return new Trigger(() -> false);
  }

  public default Trigger operatorF2() {
    return new Trigger(() -> false);
  }

  public default Trigger operatorFR1() {
    return new Trigger(() -> false);
  }

  public default Trigger operatorFR2() {
    return new Trigger(() -> false);
  }

  public default Trigger operatorFL1() {
    return new Trigger(() -> false);
  }

  public default Trigger operatorFL2() {
    return new Trigger(() -> false);
  }

  public default Trigger operatorBR1() {
    return new Trigger(() -> false);
  }

  public default Trigger operatorBR2() {
    return new Trigger(() -> false);
  }

  public default Trigger operatorBL1() {
    return new Trigger(() -> false);
  }

  public default Trigger operatorBL2() {
    return new Trigger(() -> false);
  }

  public default Trigger operatorB1() {
    return new Trigger(() -> false);
  }

  public default Trigger operatorB2() {
    return new Trigger(() -> false);
  }

  public default Trigger operatorCoralSideSwitch() {
    return new Trigger(() -> false);
  }

  public default Trigger operatorAlgaeClearingHeightSwitch() {
    return new Trigger(() -> false);
  }

  public default Trigger operatorClearAlgaeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger operatorAlgaeTargetSwitch() {
    return new Trigger(() -> false);
  }

  public default Trigger operatorFullAutoPlacementSwitch() {
    return new Trigger(() -> false);
  }
}
