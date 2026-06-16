// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;



/** Add your docs here. */
public class FieldPoints {
    DriveSubsystem Drive;
    public FieldPoints(){
        Drive = new DriveSubsystem();
    }
    
    public static Translation2d getHubPosition() {
        Optional<Alliance> allianceColor = DriverStation.getAlliance();
        if(allianceColor.isPresent() && allianceColor.get() == DriverStation.Alliance.Red) {
            return Constants.FieldConstants.Red.kHubPosition;
        }
        return Constants.FieldConstants.Blue.kHubPosition;
    }

    public static Translation2d getDepotShuttle() {
        Optional<Alliance> allianceColor = DriverStation.getAlliance();
        if(allianceColor.isPresent() && allianceColor.get() == DriverStation.Alliance.Red) {
            return Constants.FieldConstants.Red.kShuttleDepot;
        }
        return Constants.FieldConstants.Blue.kShuttleDepot;
    }

     public static Translation2d getShuttleOutpost() {
        Optional<Alliance> allianceColor = DriverStation.getAlliance();
        if(allianceColor.isPresent() && allianceColor.get() == DriverStation.Alliance.Red) {
            return Constants.FieldConstants.Red.kShuttleOutpost;
        }
        return Constants.FieldConstants.Blue.kShuttleOutpost;
     }
}
