// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.clawcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.armevator.ArmevatorPose;
import frc.robot.subsystems.claw.Claw;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClawBackwards extends Command {
  /** Creates a new ClawForward. */
  private Claw claw;

  private Supplier<ArmevatorPose> pose;

  public ClawBackwards(Claw claw, Supplier<ArmevatorPose> pose) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(claw);
    this.claw = claw;
    this.pose = pose;
  }

  public ClawBackwards(Claw claw) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(claw);
    this.claw = claw;
    this.pose = null;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (this.pose == null) {
      claw.ejectCoral();
    } else {
      switch (this.pose.get()) {
        case CORAL_L1_SCORE:
          claw.ejectCoralL1();
          break;
        case CORAL_L2_SCORE:
          claw.ejectCoralL2();
          break;
        case CORAL_L3_SCORE:
          claw.ejectCoralL3();
          break;
        case CORAL_L4_SCORE:
          claw.ejectCoral();
          break;
        default:
          claw.ejectCoral();
          break;
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw.stopClaw();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
