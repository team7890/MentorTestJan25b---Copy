// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.AlgaeTopRoller;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TopRollerRun extends Command {
  private final AlgaeTopRoller objAlgaeTopRoller;
  private final double dSpeed;
  /** Creates a new ElevatorUp. */
  public TopRollerRun(AlgaeTopRoller objAlgaeTopRoller_in, double dSpeed_In) {
    objAlgaeTopRoller = objAlgaeTopRoller_in;
    dSpeed = dSpeed_In;
    addRequirements(objAlgaeTopRoller);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  objAlgaeTopRoller.runAlgaeTopRoller(dSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    objAlgaeTopRoller.stopAlgaeTopRoller();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return false;
  }
}
