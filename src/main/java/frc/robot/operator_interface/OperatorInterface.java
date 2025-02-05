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

  public default Trigger operatorEjectAll() { // button 1
    return new Trigger(() -> false);
  }

  public default Trigger operatorVisionIsEnabledSwitch() {  // button 27
    return new Trigger(() -> false);
  }

  public default Trigger operatorResetGyroButton() { // button 2 
    return new Trigger(() -> false);
  }

  public default Trigger operatorExtendClimber() { // button 3 
    return new Trigger(() -> false);
  }

  public default Trigger operatorRetractClimber() { // button 4
    return new Trigger(() -> false);
  }

  public default Trigger operatorSlowMode() { // button 5
    return new Trigger(() -> false);
  }

  public default Trigger operatorL1() { // button 6
    return new Trigger(() -> false);
  }

  public default Trigger operatorL2() {  // button 7
    return new Trigger(() -> false);
  }

  public default Trigger operatorL3() { // button 8
    return new Trigger(() -> false);
  }

  public default Trigger operatorL4() {  // button 9
    return new Trigger(() -> false);
  }

  public default Trigger operatorF1() { // button 10
    return new Trigger(() -> false);
  }

  public default Trigger operatorF2() { // button 11
    return new Trigger(() -> false);
  }

  public default Trigger operatorFR1() {  // button 12
    return new Trigger(() -> false);
  }

  public default Trigger operatorFR2() { // button 13
    return new Trigger(() -> false);
  }

  public default Trigger operatorFL1() { // button 14
    return new Trigger(() -> false);
  }

  public default Trigger operatorFL2() { // button 15
    return new Trigger(() -> false);
  }

  public default Trigger operatorBR1() { // button 16
    return new Trigger(() -> false);
  }

  public default Trigger operatorBR2() { // button 17
    return new Trigger(() -> false);
  }

  public default Trigger operatorBL1() { // button 18
    return new Trigger(() -> false);
  }

  public default Trigger operatorBL2() { // button 19
    return new Trigger(() -> false);
  }

  public default Trigger operatorB1() { // button 20
    return new Trigger(() -> false);
  }

  public default Trigger operatorB2() { // button 21
    return new Trigger(() -> false);
  }

  public default Trigger operatorCoralSideSwitch() {  // button 22
    return new Trigger(() -> false);
  }

  public default Trigger operatorAlgaeClearingHeightSwitch() { // button 23
    return new Trigger(() -> false);
  }

  public default Trigger operatorClearAlgaeButton() { // button 24
    return new Trigger(() -> false);
  }

  public default Trigger operatorAlgaeTargetSwitch() { // button 25
    return new Trigger(() -> false);
  }

  public default Trigger operatorFullAutoPlacementSwitch() { // button 26
    return new Trigger(() -> false);
  }
}
