// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import choreo.auto.AutoChooser;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;


// === SUBSYSTEM IMPORTS === \\
import frc.robot.subsystems.AlgaeRoller;
import frc.robot.subsystems.AlgaeTopRoller;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralRoller;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.Constants.OperatorConstants;
// === COMMAND IMPORTS === \\
import frc.robot.commands.AlgaeRollerRun;
import frc.robot.commands.CoralRollerRun;
import frc.robot.commands.ElevatorRun;
import frc.robot.commands.PivotRun;
import frc.robot.commands.TeleOpDrive;
import frc.robot.commands.TopRollerRun;
import frc.robot.commands.CmndGroups.AlgaeIntake;
import frc.robot.commands.AutoDriveToAprilTag;
// === MISCELLANEOUS IMPORTS ===\\
import frc.robot.UtilityFunctions;

public class RobotContainer {

    // === VARIABLES ===\\
    private double dDriverLeftY, dDriverLeftX, dDriverRightX;

    // === SUBSYSTEMS === \\
    private final Elevator objElevator = new Elevator();
    private final AlgaeRoller objAlgaeRoller = new AlgaeRoller();
    private final AlgaeTopRoller objAlgaeTopRoller = new AlgaeTopRoller();
    private final CoralRoller objCoralRoller = new CoralRoller();
    private final Pivot objPivot = new Pivot();
    
    // === GENERATED CONTAINER === \\
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController objDriverXbox = new CommandXboxController(OperatorConstants.iDriver);
    private final CommandXboxController objCoPilotXbox = new CommandXboxController(OperatorConstants.iCoPilotPort);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // === PATHPLANNER === \\
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        // === PATHPLANNER === \\
        autoChooser = AutoBuilder.buildAutoChooser();
        NamedCommands.registerCommand("CoralIntake", new CoralRollerRun(objCoralRoller, MaxSpeed));
        SmartDashboard.putData("Auto Chooser", autoChooser);
        
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // dDriverLeftY = -objDriverXbox.getLeftY() * MaxSpeed;
        // dDriverLeftX = -objDriverXbox.getLeftX() * MaxSpeed;
        // dDriverRightX = -objDriverXbox.getRightX() * MaxAngularRate;

        // drivetrain.setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(dDriverLeftY) // Drive forward with negative Y (forward)
        //             .withVelocityY(dDriverLeftX) // Drive left with negative X (left)
        //             .withRotationalRate(dDriverRightX) // Drive counterclockwise with negative X (left)
        //     )
        // );

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            // drivetrain.applyRequest(() ->
            //     drive.withVelocityX(-objDriverXbox.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            //         .withVelocityY(-objDriverXbox.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            //         .withRotationalRate(-objDriverXbox.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)

            new TeleOpDrive(drivetrain, MaxSpeed, MaxAngularRate, ()-> -objDriverXbox.getLeftY(), ()-> -objDriverXbox.getLeftX() , ()-> -objDriverXbox.getRightX(), false)
        );

        objDriverXbox.a().whileTrue(drivetrain.applyRequest(() -> brake));
        objDriverXbox.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-objDriverXbox.getLeftY(), -objDriverXbox.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        objDriverXbox.back().and(objDriverXbox.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        objDriverXbox.back().and(objDriverXbox.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        objDriverXbox.start().and(objDriverXbox.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        objDriverXbox.start().and(objDriverXbox.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));


        // reset the field-centric heading on left bumper press
        objDriverXbox.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
        
        // === DRIVER COMMANDS === \\
   
       objDriverXbox.x().whileTrue(new CoralRollerRun(objCoralRoller, 0.55));
       objDriverXbox.y().whileTrue(new CoralRollerRun(objCoralRoller, -0.55));
    //    objDriverXbox.leftBumper().whileTrue(new TopRollerRun(objAlgaeTopRoller, 0.45));
        objDriverXbox.leftBumper().toggleOnFalse(new TeleOpDrive(drivetrain, MaxSpeed, MaxAngularRate, ()-> -objDriverXbox.getLeftY(), ()-> -objDriverXbox.getLeftX() , ()-> -objDriverXbox.getRightX(), true));

       objDriverXbox.rightBumper().whileTrue(new AutoDriveToAprilTag(drivetrain, MaxSpeed, MaxAngularRate));

       // === COPILOT COMMANDS === \\
       objCoPilotXbox.a().whileTrue(new PivotRun(objPivot, -0.15));   //DOWN
       objCoPilotXbox.x().whileTrue(new PivotRun(objPivot, 0.15));   //UP
       objCoPilotXbox.b().whileTrue(new AlgaeIntake(objAlgaeRoller, objAlgaeTopRoller, -0.55, -0.15));  //ball off reef
       objCoPilotXbox.y().whileTrue(new AlgaeIntake(objAlgaeRoller, objAlgaeTopRoller, 0.55, -0.15)); //ball off ground
       objCoPilotXbox.axisGreaterThan(5, 0.5).whileTrue(new ElevatorRun(objElevator, 0.5)); //up
       objCoPilotXbox.axisLessThan(5, -0.5).whileTrue(new ElevatorRun(objElevator, -0.5)); //down

       
    }



    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
        // return Commands.print("No autonomous command configured");
    }
}
