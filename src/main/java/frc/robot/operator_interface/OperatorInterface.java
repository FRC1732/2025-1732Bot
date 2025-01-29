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

  // drivetrain, generic

  public default double getTranslateX() {
    return 0.0;
  }

  public default double getTranslateY() {
    return 0.0;
  }

  public default double getRotate() {
    return 0.0;
  }

  public default Trigger getFieldRelativeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getResetGyroButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getResetPoseToVisionButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getXStanceButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getSlowModeSwitch() {
    return new Trigger(() -> false);
  }

  public default Trigger getLock180Button() {
    return new Trigger(() -> false);
  }

  public default Trigger getVisionIsEnabledSwitch() {
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

  public default Trigger getArmTriggerForward() {
    return new Trigger(() -> false);
  }

  public default Trigger getArmTriggerBackwards() {
    return new Trigger(() -> false);
  }

  public default Trigger getClawTriggerForwards() {
    return new Trigger(() -> false);
  }

  public default Trigger getClawTriggerBackwards() {
    return new Trigger(() -> false);
  }

  public default Trigger getIntakeCoral() {
    return new Trigger(() -> false);
  }

  public default Trigger scoreL1() {
    return new Trigger(() -> false);
  }

  public default Trigger scoreL2() {
    return new Trigger(() -> false);
  }

  public default Trigger scoreL3() {
    return new Trigger(() -> false);
  }

  public default Trigger scoreL4() {
    return new Trigger(() -> false);
  }

  // drivetrain, game-specific

  // miscellaneous
  public default Trigger getInterruptAll() {
    return new Trigger(() -> false);
  }

  public default Trigger operatorEjectAll() {
    return new Trigger(() -> false);
  } 
  
  public default Trigger operatorResetGyro() {
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

  public default Trigger operatorSetPreferedCoralSide() {
    return new Trigger(() -> false);
  } 

  public default Trigger operatorSwitchAlgaeClearingHeight() {
    return new Trigger(() -> false);
  } 

  public default Trigger operatorHoldToClearAlgae() {
    return new Trigger(() -> false);
  }

  public default Trigger operatorTargetLowOrHigh() {
    return new Trigger(() -> false);
  } 

  public default Trigger operatorToggleFullAutoPlacement() {
    return new Trigger(() -> false);
  } 
}
