package frc.robot.subsystems;

import java.util.ArrayList; // Ensure this import is complete and used in the code
import java.util.EnumMap;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.CompletableFuture;

//import frc.robot.Constants.CanIds;
//import frc.robot.commands.drivebase.DriveToPickup.TARGETPOS;
//import frc.robot.subsystems.Coral.CoralPhase;
//import frc.utils.CommonLogic;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class DriverAssist extends SubsystemBase {
    // GLOBAL VARIABLES GO BELOW THIS LINE
    private DAStatus currDAStatus = DAStatus.INIT;
    private SwerveSubsystem driveTrain;
    private Intake intake;
    private Intake.STATUS intakeStatus;
    private Launcher launcher;
    private String combinedStatus;
    private Launcher.LauncherStatus launcherStatus;
    private Pose2d targetPose;
    private Hopper hopper;
    private Hopper.HopperStatus hopperStatus;
    private Climb climb;
    private Climb.Status climbStatus;
    private FMSSystem fmsSystem;
    private FMSSystem.FMSStatus fmsStatus;
    private boolean onShift = true;

    private double remainingTime;
    private PMGT pmgt;
    private PMGT.Profiles pmgtProfile;
    private Pose2d currRobotPose; // current robot pose
    private ALLIANCEZONE currALLIANCEZONE; // current field zone
    private FIELDZONES currFIELDZONE; // current field zone
    private Pose2d SelectedDriveTargetPose2d = Targets.RRIGHTSTART.getTargetPose(); // selected target pose
    private Targets CurrDriveSelectedTarget = Targets.RRIGHTSTART; // selected target enum
    private Pose2d prevDriveTargetPose2d; // previous loop's selected target pose
    private Alliance currAlliance = Alliance.Red;
    private Alliance oppAlliance = Alliance.Blue;
    private DriveState currDriveState = DriveState.STATIONARY;
    // private CoralPhase currCoralPhase;
    // private Coral coral;
    String currCmdName = "";
    private FullState currFullState = FullState.UNKOWNSTATE;

    // enum map to store distances to each target
    private EnumMap<Targets, Double> distances = new EnumMap<>(Targets.class);

    private Targets[] TargetSets;

    // pose tolaerance values
    private static final double positionToleranceMeters = 1.0; // 10 cm tolerance
    private static final double angleToleranceRadians = Math.toRadians(5); // 5 degrees tolerance
    private boolean bAtDrivePrevTarget = false;

    // tactic variables
    private TACTIC_APPROACH currentTactic = TACTIC_APPROACH.T1;
    // private ActionStates currentActionState = ActionStates.EMPTY;

    private ONSHIFT_TACTIC onshifTactic = currentTactic.onshiftTactic;
    private OFFSHIFT_TACTIC offshifTactic = currentTactic.offshiftTactic;
    private CLIMB_TACTIC climbTactic = currentTactic.climbTactic;

    private int numofTargets = 0;

    // GLOBAL VARIABLES GO ABOVE THIS LINE
    // SYSTEM METHODS GO BELOW THIS LINE
    public DriverAssist() {
        try{
        driveTrain = RobotContainer.getInstance().m_driveTrain;
        intake = RobotContainer.getInstance().m_intake;
        launcher = RobotContainer.getInstance().m_launcher;
        hopper = RobotContainer.getInstance().m_feeder;
        fmsSystem = RobotContainer.getInstance().m_fmsSystem;
        pmgt = RobotContainer.getInstance().m_pmgt;
        climb = RobotContainer.getInstance().m_climb;
} catch (Exception e) {
    System.out.println("Error initializing DriverAssist subsystem: " + e.getMessage());
}

    if(driveTrain !=null){
        currDAStatus = DAStatus.RUNNING;

    }

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        switch (currDAStatus) {
            case RUNNING:
                getSubsystemState();
                housekeepingTasks();
                determineCurrentAction();
                determinePossibleTargets();
                determineBestTarget();
                setDriveTarget();
                break;
            case INIT:
                getSubsystems();
                

                break;

            default:
                break;
        }

        // Arm direction is positive when cmdPos is greater than curPos
        /*
         * armMotor.getClosedLoopController().setReference(armCmdPos,
         * ControlType.kMAXMotionPositionControl, ArmCurrentSlot,
         * Math.abs(Math.sin(armCurPos)) * armDirection);
         */
        // Probably should add some safety logic here
        // recommend storing new target position in a variable and then executing the
        // safety logic here.
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    public void disablePeriodic() {

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    // expose the current position
    public void autonInit() {

    }

    // SYSTEM METHODS GO ABOVE THIS LINE
    // PROCESSING METHODS GO BELOW THIS LINE
    private void getSubsystems() {
        driveTrain = RobotContainer.getInstance().m_driveTrain;

        // coral = RobotContainer.getInstance().m_Coral;

    }

    private void getSubsystemState() {

        updateDrivetrainStatus();
        intakeStatus = intake.getStatus();
        launcherStatus = launcher.getStatus();
        combinedStatus = launcher.getCombinedStatus();
        targetPose = launcher.getTargetPose();
        hopperStatus = hopper.getStatus();
        pmgtProfile = pmgt.getCurrentProfile();
        fmsStatus = fmsSystem.getStatus();
        currAlliance = fmsSystem.getAlliance();
        oppAlliance = (currAlliance == Alliance.Red) ? Alliance.Blue : Alliance.Red;
        onShift = fmsSystem.getOnShift();
        climbStatus = climb.getStatus();
        currALLIANCEZONE = getALLIANCEZONE(currRobotPose.getX());
        currFIELDZONE = getFIELDZONE(currRobotPose.getY());
        // currCoralPhase = coral.getCoralPhase();

    }

    private void housekeepingTasks() {
        // Any periodic housekeeping tasks can be done here.
        // update key status based on current location / pose robot
        prevDriveTargetPose2d = SelectedDriveTargetPose2d;
        bAtDrivePrevTarget = isPose2dCloseEnough(currRobotPose, prevDriveTargetPose2d);

        // update Tactics based upon current tactic approach.
        offshifTactic = currentTactic.offshiftTactic;
        onshifTactic = currentTactic.onshiftTactic;
        climbTactic = currentTactic.climbTactic;
        // endTactic = currentTactic.endTactic;
        // need to revise this to which Action state we are in with auto updating
        // statefully

        // check and confirm what zone we are in.
    }

    private void determineCurrentAction() {
        // Determine what action we are currently taking based on the current state of
        // the
        // subsystems

        // Should pass in the current states of all subsystems to determine the full
        // state.
        // Update search methhod with new subsystems as they are added.
        // currFullState = getCurFullState(new String[] {currDriveState.name()});
        switch (fmsStatus) {
            case PREMATCH:
                // Determine on shift action state based on subsystem states.
                break;
            case AUTONOMOUS:
                // Determine off shift action state based on subsystem states.
                break;
            case PRETELEOP:
                // Determine climb action state based on subsystem states.
                break;
            case TRANSITION:
                // Determine transition action state based on subsystem states.
                break;
            case ALLIANCE_SHIFT_1:
                // Determine alliance shift 1 action state based on subsystem states.
                break;
            case ALLIANCE_SHIFT_2:
                // Determine alliance shift 2 action state based on subsystem states.
                break;
            case ALLIANCE_SHIFT_3:
                // Determine alliance shift 3 action state based on subsystem states.
                break;
            case ALLIANCE_SHIFT_4:
                // Determine alliance shift 4 action state based on subsystem states.
                break;
            case ENDGAME:
                // Determine endgame action state based on subsystem states.
                break;
            default:
                break;
        }

        // This should be Articulation state check.
    }

    private void determinePossibleTargets() {
        // Determine what targets are closest based on the current state of the
        // subsystems. Plus other robot state information.

        distances.clear();
        TargetSets = new Targets[0];

        for (Targets target : Targets.values()) {
            Pose2d targetPose = target.getTargetPose();
            distances.put(target, getDistanceToTarget(currRobotPose, targetPose.getTranslation()));
            // Store or process the distance to the target as needed
        }

    }

    private void determineBestTarget() {
        // Determine what target is best based on the current state of the subsystems.
        // also include other robot state information.
        // also include game tactics <- need to figure out how to do this. haha

        numofTargets = distances.size(); // number of targets

        // Based on pos of robot, tactics, and shift we need to determine which drive
        // target is best.
        switch (fmsStatus) {
            case PREMATCH:
                break;
            case AUTONOMOUS:
                break;
            case PRETELEOP:
                break;
            case TRANSITION:
                determineTargetBasedOnTactic();
                break;
            case ALLIANCE_SHIFT_1:
                if (onShift) {
                    determineTargetBasedOnTactic();
                } else {
                    determineTargetBasedOffTactic();
                }
                break;
            case ALLIANCE_SHIFT_2:
                if (onShift) {
                    determineTargetBasedOnTactic();
                } else {
                    determineTargetBasedOffTactic();
                }
                break;
            case ALLIANCE_SHIFT_3:
                if (onShift) {
                    determineTargetBasedOnTactic();
                } else {
                    determineTargetBasedOffTactic();
                }
                break;
            case ALLIANCE_SHIFT_4:
                if (onShift) {
                    determineTargetBasedOnTactic();
                } else {
                    determineTargetBasedOffTactic();
                }
                break;
            case ENDGAME:
                determineTargetBasedClimbTactic();
                break;
            default:
                break;

        }

        // based on current pos of robot and shift we need to determine which launch
        // target is best.

    }

    private void setDriveTarget() {
        // Set the target for the drivebase based on the current state of the
        // subsystems.

        SelectedDriveTargetPose2d = CurrDriveSelectedTarget.getTargetPose();

    }

    private void determineLikelyActions() {
        // Determine what actions are possible based on the current state of the
        // subsystems.
        // this needs to be built out similarly to how the drive targets are setup, once
        // we determine Articulation Actions.

    }

    private void executeAction() {
        // Execute the action determined by the current state of the subsystems.
        // This may be a simple command or a complex sequence of commands.
        // Need to set articulation and drivebase targets.

        // this should be a drive target and articulation target set.

    }

    // PROCESSING METHODS GO ABOVE THIS LINE
    // SUPPORTING METHODS GO BELOW THIS LINE

    // method for storing tactic approaches
    private void reviewTacticApproach() {
        // Review the current tactic approach based on the current state of the
        // subsystems.

        // current tactic approach logic here
        // use enum TACTIC_APPROACH store current tactic approach

    }

    public void setTacticApproach(TACTIC_APPROACH tactic) {
        currentTactic = tactic;
    }

    public TACTIC_APPROACH getTacticApproach() {
        return currentTactic;
    }

    private void determineTargetBasedOnTactic() {
// Determine launch target based on current tactic approach.
if (currALLIANCEZONE.alliance == currAlliance) {
    if (launcherStatus == Launcher.LauncherStatus.IDLE) {
    targetPose = Launcher.KnownTargets.valueOf(currAlliance.name() + "_" + "HUB").getPose2d();
    }

}else {
launcher.setStatus(Launcher.LauncherStatus.IDLE);

}

        // Determine the target based on the current tactic approach.
        switch (currentTactic.onshiftTactic) {
            case RIGHTSIDE:
                // if in alliance zone, head to netural zone, else if not in alliance zone, head
                // to alliance zone
                if (currALLIANCEZONE.alliance == currAlliance) {
                    CurrDriveSelectedTarget = distances.entrySet().stream()
                            .filter(entry -> entry.getKey().getTargetType().equals("DMZ") &&
                                    entry.getKey().alliance == currAlliance &&
                                    entry.getKey().side.equals("RIGHT"))
                            .min((e1, e2) -> Double.compare(e1.getValue(), e2.getValue()))
                            .map(entry -> entry.getKey())
                            .orElse(null);

                } else {
                    // if in neutral zone, head to alliance zone
                    CurrDriveSelectedTarget = distances.entrySet().stream()
                            .filter(entry -> entry.getKey().getTargetType().equals("DRIVE") &&
                                    entry.getKey().alliance == currAlliance &&
                                    entry.getKey().side.equals("RIGHT"))
                            .min((e1, e2) -> Double.compare(e1.getValue(), e2.getValue()))
                            .map(entry -> entry.getKey())
                            .orElse(null);
                }

                break;
            case LEFTSIDE:
                // select right pickup
               
                 if (currALLIANCEZONE.alliance == currAlliance) {
                    CurrDriveSelectedTarget = distances.entrySet().stream()
                            .filter(entry -> entry.getKey().getTargetType().equals("DMZ") &&
                                    entry.getKey().alliance == currAlliance &&
                                    entry.getKey().side.equals("LEFT"))
                            .min((e1, e2) -> Double.compare(e1.getValue(), e2.getValue()))
                            .map(entry -> entry.getKey())
                            .orElse(null);

                } else {
                    // if in neutral zone, head to alliance zone
                    CurrDriveSelectedTarget = distances.entrySet().stream()
                            .filter(entry -> entry.getKey().getTargetType().equals("DRIVE") &&
                                    entry.getKey().alliance == currAlliance &&
                                    entry.getKey().side.equals("LEFT"))
                            .min((e1, e2) -> Double.compare(e1.getValue(), e2.getValue()))
                            .map(entry -> entry.getKey())
                            .orElse(null);
                }

                break;
            case CLOSE:
             if (currALLIANCEZONE.alliance == currAlliance) {
                    CurrDriveSelectedTarget = distances.entrySet().stream()
                            .filter(entry -> entry.getKey().getTargetType().equals("DMZ") &&
                                    entry.getKey().alliance == currAlliance &&
                                    entry.getKey().side.equals("CENTER"))
                            .min((e1, e2) -> Double.compare(e1.getValue(), e2.getValue()))
                            .map(entry -> entry.getKey())
                            .orElse(null);

                } else {
                    // if in neutral zone, head to alliance zone
                    CurrDriveSelectedTarget = distances.entrySet().stream()
                            .filter(entry -> entry.getKey().getTargetType().equals("CLIMB") &&
                                    entry.getKey().alliance == currAlliance &&
                                    entry.getKey().side.equals("RIGHT"))
                            .min((e1, e2) -> Double.compare(e1.getValue(), e2.getValue()))
                            .map(entry -> entry.getKey())
                            .orElse(null);
                }
                // select left pickup
              
                break;

            default:
                break;
        }

    }

    private void determineTargetBasedOffTactic() {
        // Determine the target based on the current tactic approach.
if (currALLIANCEZONE.alliance == currAlliance) {
    if (launcherStatus == Launcher.LauncherStatus.IDLE) {
 //   targetPose = Launcher.KnownTargets.valueOf(currAlliance.name() + "_" + "HUB").getPose2d();
    }

}else {
if(currFIELDZONE == FIELDZONES.RIGHTZONE) {
    targetPose = Launcher.KnownTargets.valueOf(currAlliance.name() + "_" + "OUTPOST").getPose2d();
}else if(currFIELDZONE == FIELDZONES.LEFTZONE) {
    targetPose = Launcher.KnownTargets.valueOf(currAlliance.name() + "_" + "DEPOT").getPose2d();
} else {
    targetPose = Launcher.KnownTargets.valueOf(currAlliance.name() + "_" + "CENTERTARGET").getPose2d();
}
        switch (currentTactic.offshiftTactic) {
            case RIGHTFAR:
                // find nearest deploy
                if ( (currALLIANCEZONE.alliance == currAlliance) || (currALLIANCEZONE == ALLIANCEZONE.DMZ) ) {
                    CurrDriveSelectedTarget = distances.entrySet().stream()
                            .filter(entry -> entry.getKey().getTargetType().equals("DRIVE") &&
                                    entry.getKey().alliance == oppAlliance &&
                                    entry.getKey().side.equals("LEFT"))
                            .min((e1, e2) -> Double.compare(e1.getValue(), e2.getValue()))
                            .map(entry -> entry.getKey())
                            .orElse(null);

                } else {
                    // if in neutral zone, head to alliance zone
                    CurrDriveSelectedTarget = distances.entrySet().stream()
                            .filter(entry -> entry.getKey().getTargetType().equals("DRIVE") &&
                                    entry.getKey().alliance == currAlliance &&
                                    entry.getKey().side.equals(currentTactic.onshiftTactic.getSide()))
                            .min((e1, e2) -> Double.compare(e1.getValue(), e2.getValue()))
                            .map(entry -> entry.getKey())
                            .orElse(null);
                }

                break;
                
            case LEFTFAR:
                // select right deploy
                if ( (currALLIANCEZONE.alliance == currAlliance) || (currALLIANCEZONE == ALLIANCEZONE.DMZ) ) {
                    CurrDriveSelectedTarget = distances.entrySet().stream()
                            .filter(entry -> entry.getKey().getTargetType().equals("DRIVE") &&
                                    entry.getKey().alliance == oppAlliance &&
                                    entry.getKey().side.equals("RIGHT"))
                            .min((e1, e2) -> Double.compare(e1.getValue(), e2.getValue()))
                            .map(entry -> entry.getKey())
                            .orElse(null);

                } else {
                    // if in neutral zone, head to alliance zone
                    CurrDriveSelectedTarget = distances.entrySet().stream()
                            .filter(entry -> entry.getKey().getTargetType().equals("DRIVE") &&
                                    entry.getKey().alliance == currAlliance &&
                                    entry.getKey().side.equals(currentTactic.onshiftTactic.getSide()))
                            .min((e1, e2) -> Double.compare(e1.getValue(), e2.getValue()))
                            .map(entry -> entry.getKey())
                            .orElse(null);
                }

                break;
            case CENTER:
                // select center deploy
                if ( (currALLIANCEZONE.alliance == currAlliance) || (currALLIANCEZONE.alliance == oppAlliance) ) {
                    CurrDriveSelectedTarget = distances.entrySet().stream()
                            .filter(entry -> entry.getKey().getTargetType().equals("DMZ") &&
                                    entry.getKey().alliance == oppAlliance &&
                                    entry.getKey().side.equals("CENTER"))
                            .min((e1, e2) -> Double.compare(e1.getValue(), e2.getValue()))
                            .map(entry -> entry.getKey())
                            .orElse(null);

                } else {
                    // if in neutral zone, head to alliance zone
                    CurrDriveSelectedTarget = distances.entrySet().stream()
                            .filter(entry -> entry.getKey().getTargetType().equals("DRIVE") &&
                                    entry.getKey().alliance == currAlliance &&
                                    entry.getKey().side.equals(currentTactic.onshiftTactic.getSide()))
                            .min((e1, e2) -> Double.compare(e1.getValue(), e2.getValue()))
                            .map(entry -> entry.getKey())
                            .orElse(null);
                }

                break;

            default:
                break;
        }
    }
    }

    private void determineTargetBasedClimbTactic() {
        // Determine the target based on the current tactic approach.
if (currALLIANCEZONE.alliance == currAlliance) {
    if (launcherStatus == Launcher.LauncherStatus.IDLE) {
    targetPose = Launcher.KnownTargets.valueOf(currAlliance.name() + "_" + "HUB").getPose2d();
    }

}else {
launcher.setStatus(Launcher.LauncherStatus.IDLE);

}
        switch (currentTactic.climbTactic) {
            case RIGHTCLIMB:
                // find nearest climb
               
                    CurrDriveSelectedTarget = distances.entrySet().stream()
                            .filter(entry -> entry.getKey().getTargetType().equals("CLIMB") &&
                                    entry.getKey().alliance == currAlliance &&
                                    entry.getKey().side.equals("RIGHT"))
                            .min((e1, e2) -> Double.compare(e1.getValue(), e2.getValue()))
                            .map(entry -> entry.getKey())
                            .orElse(null);

                
                break;
            case LEFTCLIMB:
                // select left climb
               CurrDriveSelectedTarget = distances.entrySet().stream()
                            .filter(entry -> entry.getKey().getTargetType().equals("CLIMB") &&
                                    entry.getKey().alliance == currAlliance &&
                                    entry.getKey().side.equals("LEFT"))
                            .min((e1, e2) -> Double.compare(e1.getValue(), e2.getValue()))
                            .map(entry -> entry.getKey())
                            .orElse(null);

                break;

            case CENTERCLIMB:
                // select center climb
                CurrDriveSelectedTarget = distances.entrySet().stream()
                            .filter(entry -> entry.getKey().getTargetType().equals("CLIMB") &&
                                    entry.getKey().alliance == currAlliance &&
                                    entry.getKey().side.equals("CENTER"))
                            .min((e1, e2) -> Double.compare(e1.getValue(), e2.getValue()))
                            .map(entry -> entry.getKey())
                            .orElse(null);
                break;
            default:
                break;

        }

    }

    public Targets[] getTargets(String type) {
        List<Targets> result = new ArrayList<Targets>();
        for (Targets targets : Targets.values()) {
            if (targets.getTargetType() == type) {
                result.add(targets);
            }
        }
        return result.toArray(new Targets[result.size()]);
    }

    // Retreive the current command of the drivetrain.
    private void updateDrivetrainStatus() {
        // Get the current status of the drivetrain.
        if (driveTrain.getCurrentCommand() != null) {
            currRobotPose = driveTrain.getPose();
            currCmdName = driveTrain.getCurrentCommand().getName();

            if (driveTrain.getSwerveDrive().getRobotVelocity().vxMetersPerSecond <= Math.abs(0.01) &&
                    driveTrain.getSwerveDrive().getRobotVelocity().vyMetersPerSecond <= Math.abs(0.01)) {
                currDriveState = DriveState.STATIONARY;
            } else {
                currDriveState = DriveState.MOVING;
            }
        }
    }

    public DriveState getCurrDriveState() {
        return currDriveState;
    }

    public Targets getCurrDriveSelectedTarget() {
        return CurrDriveSelectedTarget;
    }

    public TACTIC_APPROACH getCurrTacticApproach() {
        return currentTactic;
    }

    public boolean getbAtPrevTarg() {
        return bAtDrivePrevTarget;
    }

    public int getNumOfTargets() {
        return numofTargets;
    }

    // public ActionStates getCurrActionState() {
    // return currentActionState;
    // }

    public FullState getCurFullState(String[] args) {

        CompletableFuture<FullState> fullStateFuture = FullState.getFullRobotStateAsync(currDriveState);

        FullState result = fullStateFuture.join();
        return result;
    }

    // get angle to target in Rotation2d in degrees
    public static Rotation2d getAngleToTarget(Pose2d currentPose, Translation2d targetPosition) {
        Translation2d toTarget = targetPosition.minus(currentPose.getTranslation());
        return new Rotation2d(Math.atan2(toTarget.getY(), toTarget.getX()));
    }

    // get relative angle to target in Rotation2d in degrees
    public Rotation2d getRelativeAngleToTarget(Pose2d currentPose, Translation2d targetPosition) {
        Rotation2d angleToTarget = getAngleToTarget(currentPose, targetPosition);
        return angleToTarget.minus(currentPose.getRotation());
    }

    // get distance to target in meters
    public static double getDistanceToTarget(Pose2d currentPose, Translation2d targetPosition) {
        return currentPose.getTranslation().getDistance(targetPosition);
    }

    // SUPPORTING METHODS GO ABOVE THIS LINE
    // ENUMERATIONS GO BELOW THIS LINE
    public enum DAStatus {
        INIT, // Initialization is averything up to the point of where we are connected to our
              // subsystems.
        RUNNING, // Cheaking the data and running it.
        STOP; // The end when connection is lost.

        // private final double guideAngle;
        /*
         * GuidePos(double pivotAngle) {
         * this.guideAngle = pivotAngle;
         * }
         * 
         * public double getGuidePos() {
         * return guideAngle;
         * }
         */
    }

    public enum Targets {
        BLAUNCHLEFT(new Pose2d(3.835, 7.340, new Rotation2d(Math.toRadians(0.0))), "LAUNCH", Alliance.Blue, "LEFT"),
        BLAUNCHRIGHT(new Pose2d(3.835, 0.696, new Rotation2d(Math.toRadians(0.0))), "LAUNCH", Alliance.Blue, "RIGHT"),
        RLAUNCHLEFT(new Pose2d(12.705, 0.580, new Rotation2d(Math.toRadians(0.0))), "LAUNCH", Alliance.Red, "LEFT"),
        RLAUNCHRIGHT(new Pose2d(12.705, 7.340, new Rotation2d(Math.toRadians(0.0))), "LAUNCH", Alliance.Red, "RIGHT"),

        BFEEDLEFT(new Pose2d(2.116, 6.518, new Rotation2d(Math.toRadians(0.0))), "FEED", Alliance.Blue, "LEFT"),
        BFEEDRIGHT(new Pose2d(2.008, 1.444, new Rotation2d(Math.toRadians(0.0))), "FEED", Alliance.Blue, "RIGHT"),
        RFEEDLEFT(new Pose2d(14.524, 1.676, new Rotation2d(Math.toRadians(0.0))), "FEED", Alliance.Red, "LEFT"),
        RFEEDRIGHT(new Pose2d(14.598, 6.518, new Rotation2d(Math.toRadians(0.0))), "FEED", Alliance.Red, "RIGHT"),

        BRIGHTDMZ(new Pose2d(7.714, 1.195, new Rotation2d(Math.toRadians(90.0))), "DMZ", Alliance.Blue, "RIGHT"),
        BLEFTDMZ(new Pose2d(7.738, 6.933, new Rotation2d(Math.toRadians(-90.0))), "DMZ", Alliance.Blue, "LEFT"),

        BCENTERDMZ(new Pose2d(8.278, 4.052, new Rotation2d(Math.toRadians(0.0))), "DMZ", Alliance.Blue, "CENTER"),
        RCENTERDMZ(new Pose2d(8.278, 4.052, new Rotation2d(Math.toRadians(180.0))), "DMZ", Alliance.Red, "CENTER"),
        RRIGHTDMZ(new Pose2d(8.959, 6.687, new Rotation2d(Math.toRadians(-90.0))), "DMZ", Alliance.Red, "RIGHT"),
        RLEFTDMZ(new Pose2d(8.860, 1.444, new Rotation2d(Math.toRadians(90.0))), "DMZ", Alliance.Red, "LEFT"),

        BRIGHTSTART(new Pose2d(3.752, 0.489, new Rotation2d(Math.toRadians(90.0))), "DRIVE", Alliance.Blue, "RIGHT"),
        BLEFTSTART(new Pose2d(1.568, 3.329, new Rotation2d(Math.toRadians(0.0))), "DRIVE", Alliance.Blue, "LEFT"),
        BLEFTCLIMB(new Pose2d(18.0, 1.5, new Rotation2d(Math.toRadians(0.0))), "DRIVE", Alliance.Blue, "LEFT"),
        BRIGHTCLIMB(new Pose2d(18.0, 6.0, new Rotation2d(Math.toRadians(0.0))), "DRIVE", Alliance.Blue, "RIGHT"),

        RRIGHTSTART(new Pose2d(12.724, 7.623, new Rotation2d(Math.toRadians(-90.0))), "DRIVE", Alliance.Red, "RIGHT"),
        RLEFTSTART(new Pose2d(1.568, 4.671, new Rotation2d(Math.toRadians(180.0))), "DRIVE", Alliance.Red, "LEFT"),
        RLEFTCLIMB(new Pose2d(18.0, 6.0, new Rotation2d(Math.toRadians(0.0))), "CLIMB", Alliance.Red, "LEFT"),
        RRIGHTCLIMB(new Pose2d(14.994, 4.772, new Rotation2d(Math.toRadians(180.0))), "CLIMB", Alliance.Red, "RIGHT");

        private final Pose2d targetPose;
        private final String targetType;
        private Alliance alliance;
        private final String side;

        Targets(Pose2d targetPose, String targetType, Alliance alliance, String side) {
            this.targetPose = targetPose;
            this.targetType = targetType;
            this.alliance = alliance;
            this.side = side;

        }

        public Pose2d getTargetPose() {
            return targetPose;
        }

        public String getTargetType() {
            return targetType;
        }
    }

    // system State based on drive train and other subsystems
    public enum FullState {
        STATIONARY(DriveState.STATIONARY),
        MOVING(DriveState.MOVING),
        UNKOWNSTATE(DriveState.STATIONARY);

        private final DriveState driveTrainState;

        FullState(DriveState driveTrainState) {
            this.driveTrainState = driveTrainState;
        }

        public DriveState getDriveState() {
            return driveTrainState;
        }

        public static CompletableFuture<FullState> getFullRobotStateAsync(DriveState driveTrainState) {
            return CompletableFuture.supplyAsync(() -> {
                for (FullState state : FullState.values()) {
                    if (state.getDriveState() == driveTrainState) {
                        return state;
                    }
                }
                return UNKOWNSTATE;

            });
        }
    }

    // state of DriveTrain subsystem
    public enum DriveState {
        STATIONARY,
        MOVING;
    }

    public enum FIELDZONES {
        RIGHTZONE (0.0, 7.62),
        LEFTZONE (7.62, 16.4592),
        CENTERZONE (7.62, 8.2296);

        private double start;
        private double end;
        FIELDZONES(double start, double end) {
            this.start = start;
            this.end = end;
        }
        public boolean isInZone(double y) {
            return y >= start && y <= end;
        }
    }
    
    public FIELDZONES getFIELDZONE(double y) {
        for (FIELDZONES zone : FIELDZONES.values()) {
            if (zone.isInZone(y)) {
                return zone;
            }
        }
        return null; // or throw an exception if y is out of bounds
    }

    public enum ALLIANCEZONE {
        REDZONE(0.0, 8.2296, Alliance.Red),
        BLUEZONE(8.2296, 16.4592, Alliance.Blue),
        DMZ(7.62, 8.2296, null);

        double start;
        double end;
        Alliance alliance;

        ALLIANCEZONE(double start, double end, Alliance alliance) {
            this.start = start;
            this.end = end;
            this.alliance = alliance;
        }

        public boolean isInZone(double x) {
            return x >= start && x <= end;
        }

    }

    public ALLIANCEZONE getALLIANCEZONE(double x) {
        for (ALLIANCEZONE zone : ALLIANCEZONE.values()) {
            if (zone.isInZone(x)) {
                return zone;
            }
        }
        return null; // or throw an exception if x is out of bounds
    }

    public enum TRANSITION_TACTIC {

    }

    public enum SHIFT1_TACTIC {

    }

    public enum SHIFT2_TACTIC {

    }

    public enum SHIFT3_TACTIC {

    }

    public enum SHIFT4_TACTIC {

    }

    public enum ENDGAME_TACTIC {

    }

    // Tactic Approaches pickup
    public enum ONSHIFT_TACTIC {
        LEFTSIDE ("LEFT"),
        RIGHTSIDE ("RIGHT"),
        CLOSE ("CENTER");

        private String side;
        ONSHIFT_TACTIC(String side) {
            this.side = side;
        }
        public String getSide() {
            return side;
        }
    }

    public enum OFFSHIFT_TACTIC {
        LEFTFAR,
        RIGHTFAR,
        CENTER;
    }

    public enum CLIMB_TACTIC {
        LEFTCLIMB,
        RIGHTCLIMB,
        CENTERCLIMB;
    }

    public enum TACTIC_APPROACH {
        T1(ONSHIFT_TACTIC.RIGHTSIDE, OFFSHIFT_TACTIC.RIGHTFAR, CLIMB_TACTIC.RIGHTCLIMB),
        T2(ONSHIFT_TACTIC.LEFTSIDE, OFFSHIFT_TACTIC.LEFTFAR, CLIMB_TACTIC.LEFTCLIMB),
        T3(ONSHIFT_TACTIC.CLOSE, OFFSHIFT_TACTIC.CENTER, CLIMB_TACTIC.CENTERCLIMB);

        private ONSHIFT_TACTIC onshiftTactic;
        private OFFSHIFT_TACTIC offshiftTactic;
        private CLIMB_TACTIC climbTactic;

        TACTIC_APPROACH(ONSHIFT_TACTIC onshift, OFFSHIFT_TACTIC offshift, CLIMB_TACTIC climb) {
            this.onshiftTactic = onshift;
            this.offshiftTactic = offshift;
            this.climbTactic = climb;
        }

        public ONSHIFT_TACTIC getOnshiftTactic() {
            return onshiftTactic;
        }

        public OFFSHIFT_TACTIC getOffshiftTactic() {
            return offshiftTactic;
        }

        public CLIMB_TACTIC getClimbTactic() {
            return climbTactic;
        }

    }

    /*
     * public enum ActionStates {
     * STOWED,
     * PICKINGUP,
     * DEPLOYING,
     * EMPTY,
     * CLIMB_PREP,
     * CLIMB,
     * UNKNOWN;
     * }
     */

    // ENUMERATIONS GO ABOVE THIS LINE
    // REFERENCE METHODS GO BELOW THIS LINE

    /**
     * Checks if two poses are within given tolerances.
     *
     * @param currentPose             The robot's current pose
     * @param targetPose              The desired target pose
     * @param positionToleranceMeters Allowed distance tolerance (meters)
     * @param angleToleranceRadians   Allowed angle tolerance (radians)
     * @return true if within tolerance, false otherwise
     */
    public static boolean isPose2dCloseEnough(Pose2d currentPose, Pose2d targetPose) {
        // Calculate translation distance
        double dx = currentPose.getX() - targetPose.getX();
        double dy = currentPose.getY() - targetPose.getY();
        double distance = Math.hypot(dx, dy);

        // Calculate angular difference
        double angleDiff = Math.abs(
                currentPose.getRotation().minus(targetPose.getRotation()).getRadians());

        return (distance <= positionToleranceMeters) && (angleDiff <= angleToleranceRadians);
    }

    public static double CapMotorPower(double MotorPower, double negCapValue, double posCapValue) {
        // logic to cap the motor power between a good range
        double retValue = MotorPower;

        if (MotorPower < negCapValue) {
            retValue = negCapValue;
        }

        if (MotorPower > posCapValue) {
            retValue = posCapValue;
        }

        return retValue;
    }

    public static final double joyDeadBand(double joy, double deadband) {

        double retValue = joy;
        if (Math.abs(retValue) < Math.abs(deadband)) {
            retValue = 0;
        }
        return Math.pow(retValue, 2) * Math.signum(joy);
    }

    public static final boolean isInRange(double curValue, double targetValue, double Tol) {

        double loVal = targetValue - Tol;
        double hiVal = targetValue + Tol;
        boolean retValue = false;

        if (curValue > loVal && curValue < hiVal) {
            retValue = true;
        }
        return retValue;
    }

    public static double getTimeInSeconds() {
        return (System.nanoTime() / Math.pow(10, 9)); // returns time in seconds
    }

    public double deg2Radians(double degrees) {
        return Math.toRadians(degrees);
    }

    public static double calcArcLength(double degrees, double radius) {
        return (Math.toRadians(degrees) * radius);
    }

    public static double gotoPosPIDF(double P, double F_hold, double currentPos, double targetPos) {
        double delta = targetPos - currentPos;

        return (delta * P) + F_hold;

    }

    public static double AutoTurnPIDF(double P, double F_hold, double currentPos, double targetPos) {
        double delta = targetPos - currentPos;
        if (delta > 180) {
            delta = -delta;
        }

        return (delta * P) + F_hold;

    }

    public static int getIsBlue() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                // is blue = false
                return 0;
            }
            if (ally.get() == Alliance.Blue) {
                // is blue = true
                return 1;
            }
        } else {
            // naptime
            return -1;
        }
        return -1;
    }

    public static double getTimeInMIlliseconds() {
        return (System.nanoTime() / Math.pow(10, 6));
    }

    public static double headingDeltaInDegree(double currentHeading, double targetHeading) {
        double headingDelta = 0;
        double invertedHeadingDelta = 0;

        // Positive value
        if (currentHeading >= 0 && targetHeading >= 0) {
            headingDelta = targetHeading - currentHeading;
        }
        // one of each
        else if (currentHeading >= 0 && targetHeading <= 0) {
            // headingDelta = (targetHeading + currentHeading);
            headingDelta = Math.abs(targetHeading) + Math.abs(currentHeading);
            invertedHeadingDelta = Math.abs(360 + targetHeading) - Math.abs(currentHeading);
            headingDelta = Math.min(Math.abs(headingDelta), Math.abs(invertedHeadingDelta));
            if (invertedHeadingDelta != headingDelta) {
                headingDelta = headingDelta * -1;
            }
        }
        // one of each again
        else if (currentHeading <= 0 && targetHeading >= 0) {
            // headingDelta = -1 * (targetHeading + currentHeading);
            headingDelta = Math.abs(targetHeading) + Math.abs(currentHeading);
            invertedHeadingDelta = Math.abs(360 - targetHeading) - Math.abs(currentHeading);
            headingDelta = Math.min(Math.abs(headingDelta), Math.abs(invertedHeadingDelta));
            if (invertedHeadingDelta == headingDelta) {
                headingDelta = headingDelta * -1;
            }
        }
        // both negative
        else if (currentHeading <= 0 && targetHeading <= 0) {
            headingDelta = targetHeading - currentHeading;
        }
        return headingDelta;
    }

    public static double calcTurnRateAbs(double currentHeading, double targetHeading, double proportion) {

        double headingDelta = headingDeltaInDegree(currentHeading, targetHeading);

        double commandedTurnRate = headingDelta * proportion;
        return commandedTurnRate; // IS ALWAYS POSITIVE!
    }

    public static double metersToInches(double meter) {
        return meter / 0.0254;
    }

    public static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }

    // REFERENCE METHODS GO ABOVE THIS LINE

}
