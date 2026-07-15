package frc.robot;

/**
 * RobotContainer_PIDTuning_Additions
 * 
 * This is NOT a standalone file — it shows the lines to add to your
 * existing RobotContainer.java to wire up the PID tuning system.
 * 
 * Search for the SmartDashboard.putData block near the bottom of your
 * RobotContainer constructor and add the lines from the section below.
 */

/*

// -------------------------------------------------------------------------
// ADD THESE IMPORTS at the top of RobotContainer.java
// -------------------------------------------------------------------------

import frc.robot.commands.TunePID;
import frc.robot.commands.AutoTuneMultiMotor;


// -------------------------------------------------------------------------
// ADD THESE LINES inside your RobotContainer constructor,
// alongside your existing SmartDashboard.putData() calls
// -------------------------------------------------------------------------

// Manual tuning — tweak kP/kI/kD/Setpoint live from the dashboard.
// Only one can run at a time; clicking a second button cancels the first.
SmartDashboard.putData("Tune Intake PID",   new TunePID(m_intake,   m_intake));
SmartDashboard.putData("Tune Launcher PID", new TunePID(m_launcher, m_launcher));
SmartDashboard.putData("Tune Hopper PID",   new TunePID(m_feeder,   m_feeder));

// Auto-tuning — Ziegler-Nichols. Tunes each motor independently and
// publishes recommended kP/kI/kD to SmartDashboard when complete.
// WARNING: the mechanism will oscillate during this process.
// Run in open space with software limits enabled.
SmartDashboard.putData("AutoTune Flywheel LH",
    new AutoTuneMultiMotor(m_launcher.flywheels_LH, m_launcher.flywheels_LH, 3750.0));

SmartDashboard.putData("AutoTune Flywheel RH",
    new AutoTuneMultiMotor(m_launcher.flywheels_RH, m_launcher.flywheels_RH, 3750.0));


// -------------------------------------------------------------------------
// SMARTDASHBOARD KEYS YOU WILL SEE DURING / AFTER TUNING
// -------------------------------------------------------------------------
//
// Manual TunePID (example for Intake):
//   Intake/kP           <- set this
//   Intake/kI           <- set this
//   Intake/kD           <- set this
//   Intake/Setpoint     <- set this
//   Intake/Measurement  <- read only
//   Intake/Error        <- read only
//   Intake/Output       <- read only
//   Intake/AtSetpoint   <- read only
//
// AutoTuneMultiMotor (example for flywheels_LH):
//   LFlywheels/AutoTune/Status              <- current phase
//   LFlywheels/Left/Measurement             <- live RPM
//   LFlywheels/Left/CurrentKp              <- kP being tested
//   LFlywheels/Left/OscillationCount       <- cycles detected
//   LFlywheels/Left/Recommended_kP         <- result
//   LFlywheels/Left/Recommended_kI         <- result
//   LFlywheels/Left/Recommended_kD         <- result
//   LFlywheels/Left/UltimateGain_Ku        <- raw Ziegler-Nichols value
//   LFlywheels/Left/OscillationPeriod_Tu   <- raw Ziegler-Nichols value
//   LFlywheels/Right/...                   <- same keys for right motor
//   LFlywheels/AutoTune/Summary            <- human readable final output


// -------------------------------------------------------------------------
// AFTER TUNING — updating your gains permanently
// -------------------------------------------------------------------------
//
// The auto-tuner uses kNoPersistParameters, so gains are NOT saved to
// flash automatically. Once you are happy with the recommended values:
//
// 1. Copy the recommended kP/kI/kD from SmartDashboard
// 2. Update your flyWheelLeft / flyWheelRight arrays in Constants.java
//    (or wherever you define them before passing to the LFlywheels constructor)
// 3. Redeploy — the new values will be written with kPersistParameters
//    on the next robot startup via configLTowerMotor() / configrRTowerMotor()
//
// Remember: treat Ziegler-Nichols values as a starting point.
// Back off kP and kD by ~20% before competition use.
// kF (feedforward) must be characterized separately using WPILib SysId.

*/
