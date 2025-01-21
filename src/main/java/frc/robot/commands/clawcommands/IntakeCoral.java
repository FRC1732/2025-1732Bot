// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.clawcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.rgb.StatusRgb;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoral extends Command {
  /** Creates a new IntakeCoral. */
  private Claw claw;
  private StatusRgb statusRgb;

  public IntakeCoral(Claw claw, StatusRgb statusRgb) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(claw);
    this.claw = claw;
    this.statusRgb = statusRgb;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    claw.runClaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw.stopClaw();
    if (claw.hasCoral()) {
      statusRgb.acquiredCoral();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return claw.hasCoral();
  }
}
