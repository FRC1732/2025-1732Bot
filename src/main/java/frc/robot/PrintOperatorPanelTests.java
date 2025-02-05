// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.operator_interface.OperatorInterface;

/** Add your docs here. */
public class PrintOperatorPanelTests {
    public PrintOperatorPanelTests(OperatorInterface oi) {

        oi.operatorEjectAll().whileTrue(new PrintCommand("Eject All"));
        oi.operatorVisionIsEnabledSwitch().whileTrue(new PrintCommand("Vision"));
        oi.operatorResetGyroButton().whileTrue(new PrintCommand("Gyro"));
        oi.operatorExtendClimber().whileTrue(new PrintCommand("Extend Climber"));
        oi.operatorRetractClimber().whileTrue(new PrintCommand("Retract Climber"));
        oi.operatorSlowMode().whileTrue(new PrintCommand("Slow Mode"));
        oi.operatorL1().whileTrue(new PrintCommand("L1"));
        oi.operatorL2().whileTrue(new PrintCommand("L2"));
        oi.operatorL3().whileTrue(new PrintCommand("L3"));
        oi.operatorL4().whileTrue(new PrintCommand("L4"));
        oi.operatorF1().whileTrue(new PrintCommand("F1"));
        oi.operatorF2().whileTrue(new PrintCommand("F2"));
        oi.operatorFR1().whileTrue(new PrintCommand("FR1"));
        oi.operatorFR2().whileTrue(new PrintCommand("FR2"));
        oi.operatorFL1().whileTrue(new PrintCommand("FL1"));
        oi.operatorFL2().whileTrue(new PrintCommand("FL2"));
        oi.operatorBR1().whileTrue(new PrintCommand("BR1"));
        oi.operatorBR2().whileTrue(new PrintCommand("BR2"));
        oi.operatorBL1().whileTrue(new PrintCommand("BL1"));
        oi.operatorBL2().whileTrue(new PrintCommand("BL2"));
        oi.operatorB1().whileTrue(new PrintCommand("B1"));
        oi.operatorB2().whileTrue(new PrintCommand("B2"));
        oi.operatorCoralSideSwitch().whileTrue(new PrintCommand("Side Switch"));
        oi.operatorAlgaeClearingHeightSwitch().whileTrue(new PrintCommand("Height Switch"));
        oi.operatorClearAlgaeButton().whileTrue(new PrintCommand("Clear Algae"));
        oi.operatorAlgaeTargetSwitch().whileTrue(new PrintCommand("Target Switch"));
        oi.operatorFullAutoPlacementSwitch().whileTrue(new PrintCommand("Auto Placement"));

    }
}
