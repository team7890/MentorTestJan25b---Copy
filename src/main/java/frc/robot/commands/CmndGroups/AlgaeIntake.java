// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CmndGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.AlgaeRollerRun;
import frc.robot.commands.TopRollerRun;
import frc.robot.subsystems.AlgaeRoller;
import frc.robot.subsystems.AlgaeTopRoller;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlgaeIntake extends ParallelCommandGroup {
  /** Creates a new AlgaeIntake. */
  public AlgaeIntake(AlgaeRoller objAlgaeRoller, AlgaeTopRoller objAlgaeTopRoller, double dBttmRollerSpeed, double dTopRollerSpeed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AlgaeRollerRun(objAlgaeRoller, dBttmRollerSpeed),
      new TopRollerRun(objAlgaeTopRoller, dTopRollerSpeed)
    );
  }
}
