package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class DynamicCommand extends Command {
  private final Supplier<Command> commandSupplier;
  private Command innerCommand;

  /**
   * Creates a DynamicCommand that will generate its inner command at initialization.
   *
   * @param commandSupplier A Supplier that returns a new Command instance when called.
   */
  public DynamicCommand(Supplier<Command> commandSupplier) {
    this.commandSupplier = commandSupplier;
    // Note: We do not call addRequirements() here
  }

  @Override
  public void initialize() {
    innerCommand = commandSupplier.get();
    innerCommand.initialize();
  }

  @Override
  public void execute() {
    if (innerCommand != null) {
      innerCommand.execute();
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (innerCommand != null) {
      innerCommand.end(interrupted);
    }
  }

  @Override
  public boolean isFinished() {
    return innerCommand != null && innerCommand.isFinished();
  }
}
