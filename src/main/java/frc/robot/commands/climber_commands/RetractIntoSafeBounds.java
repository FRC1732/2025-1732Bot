// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber_commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber_subsystem.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RetractIntoSafeBounds extends Command {
  /** Creates a new RetractIntoSafeBounds. */
  private Climber climber;

  private boolean runBackwards;

  private Timer timer;
  private double retractTime;

  public RetractIntoSafeBounds(Climber climber, boolean backwards, double time) {
    this.climber = climber;
    addRequirements(climber);

    timer = new Timer();

    runBackwards = backwards;
    retractTime = time;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (runBackwards) {
      climber.reverseClimber();
    } else {
      climber.runClimber();
    }

    climber.setRunningReturnCommand(true);

    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stopClimber();
    climber.setRunningReturnCommand(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(retractTime);
  }
}
