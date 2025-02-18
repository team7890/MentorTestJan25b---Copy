// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeRoller;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeRollerRun extends Command {
  private final AlgaeRoller objAlgaeRoller;
  private final double dSpeed;
  /** Creates a new ElevatorUp. */
  public AlgaeRollerRun(AlgaeRoller objAlgaeRoller_in, double dSpeed_In) {
    objAlgaeRoller = objAlgaeRoller_in;
    dSpeed = dSpeed_In;
    addRequirements(objAlgaeRoller);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  objAlgaeRoller.runAlgaeRoller(dSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    objAlgaeRoller.stopAlgaeRoller();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return false;
  }
}
