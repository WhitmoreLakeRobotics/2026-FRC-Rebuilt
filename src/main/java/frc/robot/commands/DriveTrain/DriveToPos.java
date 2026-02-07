package frc.robot.commands.DriveTrain;

import javax.swing.RootPaneContainer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;

public class DriveToPos extends Command {
    private boolean bDone = false;
    private double bHeading;
    private double rHeading;
    private int latestID;
    private Pose2d newTarget;
    private boolean isLeft = true;
    private boolean isBlue = false;

    
    public DriveToPos(boolean isLeft2) {
        this.isLeft = isLeft2;
        // m_subsystem = subsystem;
        // addRequirements(m_subsystem);

        

    }
    public DriveToPos(boolean isLeft2, boolean isBlue) {
        this.isLeft = isLeft2;
        this.isBlue = isBlue;
        // m_subsystem = subsystem;
        // addRequirements(m_subsystem);

        

    }
    public DriveToPos(Pose2d newPose2d) {
        this.newTarget = newPose2d;
        // m_subsystem = subsystem;
        // addRequirements(m_subsystem);
        
    }
    // if fixedDist = false => stagPosition is suposed to recieve the percantage to
    // be traversed in stag, in 0.xx format

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        bDone = false;
        
   
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        CommandScheduler.getInstance().schedule(RobotContainer.getInstance().m_driveTrain.driveToPose(newTarget)); 
        bDone = true;
        end(false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        CommandScheduler.getInstance().cancel(RobotContainer.getInstance().m_driveTrain.driveToPose(newTarget));
        bDone = true;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return bDone;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;

    }
    public Pose2d returnPose2d(int targetID, boolean isLeft) {
        Pose2d newDestination;
        if (isLeft) {
          
        
        switch (targetID) {
          case (8):
            //newDestination = new Pose2d(new Translation2d(13.756, 5.262), Rotation2d.fromDegrees(63.80));
            newDestination = TARGETPOS.POS8.getLefPose2d();
            break;
          case (9):
            //newDestination = new Pose2d(new Translation2d(12.367, 5.141), Rotation2d.fromDegrees(122.27));
            newDestination = TARGETPOS.POS9.getLefPose2d();
            break;
          case (10):
            //newDestination = new Pose2d(new Translation2d(11.747, 4.180), Rotation2d.fromDegrees(0.0));
            newDestination = TARGETPOS.POS10.getLefPose2d();
            break;
          case (11):
            //newDestination = new Pose2d(new Translation2d(12.285, 2.953), Rotation2d.fromDegrees(61.193));
            newDestination = TARGETPOS.POS11.getLefPose2d();
            break;
          case (12):
            //newDestination = new Pose2d(new Translation2d(13.571, 2.794), Rotation2d.fromDegrees(117.225));
            newDestination = TARGETPOS.POS6.getLefPose2d();
            break;
          case (7):
            //newDestination = new Pose2d(new Translation2d(14.399, 3.831), Rotation2d.fromDegrees(180.0));
            newDestination = TARGETPOS.POS7.getLefPose2d();
            break;
        
          default:
          //newDestination = new Pose2d(swerveDrive.getPose().getTranslation(), swerveDrive.getPose().getRotation());
          //newDestination = new Pose2d(new Translation2d(16.0, 4.5), Rotation2d.fromDegrees(0));
          newDestination = TARGETPOS.DEFAULT.getLefPose2d();
        }
        } else {
    
        switch (targetID) {
          case (8):
            //newDestination = new Pose2d(new Translation2d(13.756, 5.262), Rotation2d.fromDegrees(63.80));
            newDestination = TARGETPOS.POS8.getRightPose2d();
            break;
          case (9):
            //newDestination = new Pose2d(new Translation2d(12.367, 5.141), Rotation2d.fromDegrees(122.27));
            newDestination = TARGETPOS.POS9.getRightPose2d();
            break;
          case (10):
            //newDestination = new Pose2d(new Translation2d(11.747, 4.180), Rotation2d.fromDegrees(0.0));
            newDestination = TARGETPOS.POS10.getRightPose2d();
            break;
          case (11):
            //newDestination = new Pose2d(new Translation2d(12.285, 2.953), Rotation2d.fromDegrees(61.193));
            newDestination = TARGETPOS.POS11.getRightPose2d();
            break;
          case (12):
            //newDestination = new Pose2d(new Translation2d(13.571, 2.794), Rotation2d.fromDegrees(117.225));
            newDestination = TARGETPOS.POS6.getRightPose2d();
            break;
          case (7):
            //newDestination = new Pose2d(new Translation2d(14.399, 3.831), Rotation2d.fromDegrees(180.0));
            newDestination = TARGETPOS.POS7.getRightPose2d();
            break;
        
          default:
          //newDestination = new Pose2d(swerveDrive.getPose().getTranslation(), swerveDrive.getPose().getRotation());
          //newDestination = new Pose2d(new Translation2d(16.0, 4.5), Rotation2d.fromDegrees(0));
          newDestination = TARGETPOS.DEFAULT.getRightPose2d();
        }
        
      
      }
        return newDestination;
      }

    public Pose2d returnPosePickup(boolean isLeft,int isBlue) {
    if(isLeft){
      if(isBlue == 1){
        //is left and blue
        return TARGETPOS.BPICKUP.leftPose2d;
      }else {  //is red  
        //is left and red
        return TARGETPOS.PICKUP.leftPose2d;
      }
    }else{  //is right
      if(isBlue == 1){
        //is right and blue
        return TARGETPOS.BPICKUP.rightPose2d;
      }else {  //is red
        //is right and red
        return TARGETPOS.PICKUP.rightPose2d;
        
      }
    }


  }

  public enum TARGETPOS {
    POS8(new Pose2d(new Translation2d(13.861, 5.067), Rotation2d.fromDegrees(60.0)),
          new Pose2d(new Translation2d(13.571, 5.237), Rotation2d.fromDegrees(60.0))),
    POS9(new Pose2d(new Translation2d(12.524, 5.227), Rotation2d.fromDegrees(120.000)),
          new Pose2d(new Translation2d(12.255, 5.067), Rotation2d.fromDegrees(120.000))),
    POS10(new Pose2d(new Translation2d(11.747, 4.180), Rotation2d.fromDegrees(180.0)),
           new Pose2d(new Translation2d(11.747, 3.870), Rotation2d.fromDegrees(180.0))),
    POS11(new Pose2d(new Translation2d(12.285, 2.953), Rotation2d.fromDegrees(-120.000)),
           new Pose2d(new Translation2d(12.534, 2.823), Rotation2d.fromDegrees(-120.000))),
    POS6(new Pose2d(new Translation2d(13.571, 2.794), Rotation2d.fromDegrees(-60.0)),
            new Pose2d(new Translation2d(13.831, 2.953), Rotation2d.fromDegrees(-60.0))),
    POS7(new Pose2d(new Translation2d(14.389, 3.870), Rotation2d.fromDegrees(0.0)),
          new Pose2d(new Translation2d(14.389, 4.190), Rotation2d.fromDegrees(0.0))),
    PICKUP(new Pose2d(new Translation2d(15.915, 0.620), Rotation2d.fromDegrees(-60.00)),
          new Pose2d(new Translation2d(15.951, 7.400),Rotation2d.fromDegrees(60.00))),



    BPOS17(new Pose2d(new Translation2d(3.600, 2.973), Rotation2d.fromDegrees(-120.00)),
          new Pose2d(new Translation2d(3.959, 2.853), Rotation2d.fromDegrees(-120.00))),
    BPOS18(new Pose2d(new Translation2d(3.151, 4.190), Rotation2d.fromDegrees(180.0)),
          new Pose2d(new Translation2d(3.151, 3.880), Rotation2d.fromDegrees(180.0))),
    BPOS19(new Pose2d(new Translation2d(3.999, 5.256), Rotation2d.fromDegrees(120.000)),
          new Pose2d(new Translation2d(3.680, 5.087), Rotation2d.fromDegrees(120.000))),
    BPOS20(new Pose2d(new Translation2d(11.747, 4.180), Rotation2d.fromDegrees(60.0)),
           new Pose2d(new Translation2d(11.747, 3.930), Rotation2d.fromDegrees(60.0))),
    BPOS21(new Pose2d(new Translation2d(5.833, 3.870), Rotation2d.fromDegrees(0.0)),
          new Pose2d(new Translation2d(5.823, 4.16), Rotation2d.fromDegrees(0.00))),
    BPOS22(new Pose2d(new Translation2d(4.996, 2.794), Rotation2d.fromDegrees(-60.000)), 
            new Pose2d(new Translation2d(5.275, 3.003), Rotation2d.fromDegrees(-60.000))),
    BPICKUP(new Pose2d(new Translation2d(1.685, 7.380), Rotation2d.fromDegrees(120.00)),
            new Pose2d(new Translation2d(1.604, 0.693),Rotation2d.fromDegrees(-120.00))),
    DEFAULT(new Pose2d(new Translation2d(16.0, 4.5), Rotation2d.fromDegrees(0)),
              new Pose2d(new Translation2d(16.0, 4.5), Rotation2d.fromDegrees(0)));
      





    private Pose2d leftPose2d;
    private Pose2d rightPose2d;


    TARGETPOS(Pose2d leftPose2d, Pose2d rightPose2d){
      this.leftPose2d = leftPose2d;
      this.rightPose2d = rightPose2d;
    }


    public Pose2d getLefPose2d(){
      return leftPose2d;

    }
    public Pose2d getRightPose2d(){
      return rightPose2d;

    }
  }
}