package frc.robot.commands.DriveTrain;

import java.io.File;

import javax.swing.RootPaneContainer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToPos extends Command {
    
    private Pose2d newTarget;
   

    private Command dPos;
    private Subsystem m_subsystem;

    
    
    public DriveToPos(Pose2d newPose2d, Subsystem subsystem) {
        this.newTarget = newPose2d;
         this.m_subsystem = subsystem;
         addRequirements(m_subsystem);
        
    }
    // if fixedDist = false => stagPosition is suposed to recieve the percantage to
    // be traversed in stag, in 0.xx format

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
      
        dPos = RobotContainer.getInstance().m_driveTrain.driveToPose(newTarget); 
    
      dPos.initialize(); 

   
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      dPos.execute();
       

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      dPos.end(interrupted);
      
    
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return dPos.isFinished(); // bDone
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;

    }


    
}