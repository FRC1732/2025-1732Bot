// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.L1commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.L1Scorer.L1Scorer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeL1 extends Command {
  /** Creates a new IntakeL1. */
  private L1Scorer l1Scorer;

  public IntakeL1(L1Scorer l1Scorer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.l1Scorer = l1Scorer;
    addRequirements(l1Scorer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    l1Scorer.runIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    l1Scorer.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
