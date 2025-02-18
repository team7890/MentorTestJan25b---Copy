// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.LimelightHelpers;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

//dError = TX - dTarget
//dControl = dError * dKp

public class AutoDriveToAprilTag extends Command {
  
  private final CommandSwerveDrivetrain objSwerve;
  // private final LimelightHelpers objLimelight = new LimelightHelpers();
  private final double dMaxSpeed;
  private final double dMaxAngularRate;
  double dLimeTX, dError, dKp, dControl, dIntegral, dKi;
 
  private SwerveRequest.FieldCentric drive  = new SwerveRequest.FieldCentric()
  .withDeadband(0.0).withRotationalDeadband(0.0) // Add a 10% deadband
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors;

  // = new SwerveRequest.FieldCentric()
  // .withDeadband(dMaxSpeed * 0.2).withRotationalDeadband(dMaxAngularRate * 0.15) // Add a 10% deadband
  // .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors


  /** Creates a new AutoDriveToAprilTag. */
  // public AutoDriveToAprilTag(CommandSwerveDrivetrain objSwerve_in, LimelightHelpers objLimelight_in) {
  public AutoDriveToAprilTag(CommandSwerveDrivetrain objSwerve_in, double dMaxSpeed_in, double dMaxAngularRate_in) {
    objSwerve = objSwerve_in;
    dMaxSpeed = dMaxSpeed_in;
    dMaxAngularRate = dMaxAngularRate_in;
    
    addRequirements(objSwerve);
    // Use addRequirements() here to declare subsystem dependencies.

    
  }
 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dIntegral = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dLimeTX = LimelightHelpers.getTX("");
    dError = dLimeTX - 0.0;   //TODO: Change if needed
    dIntegral = dIntegral + dError;
    dKi = 0.175/100.0;
    dKp = 0.2/10.0;
    dControl = -dError * dKp + -dIntegral * dKi;
      
    objSwerve.setControl(
          drive.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(dControl)
      );
    
    // System.out.println("AutoDriveToAprilTag");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    objSwerve.setControl(
        drive.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0)
    );
}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return Math.abs(objLimelight.getTargetAngle()) < 1.0;
    return false;
  }
}