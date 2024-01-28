// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.io;

import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

// Trying something different this year, instead of a normal class
// the dashboard this year is using a singleton class, so only one instance will
// run at once
public class Dashboard {
    private static Dashboard INSTANCE;
    private final NetworkTableInstance ntinst;
    private final NetworkTable rPiTable;

    private final SendableChooser<DriveMode> driveModeChooser;

    public static Dashboard getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Dashboard();
        }

        return INSTANCE;
    }

    private Dashboard() {

        ntinst = NetworkTableInstance.getDefault();
        rPiTable = ntinst.getTable("RaspberryPi");

        //Creates options for different choosers

        driveModeChooser = new SendableChooser<DriveMode>();
        driveModeChooser.addOption(DriveMode.FIELD_CENTRIC.name(), DriveMode.FIELD_CENTRIC);
        driveModeChooser.addOption(DriveMode.ROBOT_CENTRIC.name(), DriveMode.ROBOT_CENTRIC);
        driveModeChooser.setDefaultOption(DriveMode.FIELD_CENTRIC.name(), DriveMode.FIELD_CENTRIC);
        SmartDashboard.putData(Constants.Dashboard.DRIVE_MODE_KEY, driveModeChooser);
    }

    public <V> void putData(String key, V value) {
        if (value instanceof Float) {
            SmartDashboard.putNumber(key, (Float) value);
        } else if (value instanceof Integer) {
            SmartDashboard.putNumber(key, (Integer) value);
        } else if (value instanceof Number) {
            SmartDashboard.putNumber(key, (Double) value);
        } else if (value instanceof String) {
            SmartDashboard.putString(key, (String) value);
        } else if (value instanceof Boolean) {
            SmartDashboard.putBoolean(key, (Boolean) value);
        } else if (value instanceof Sendable) {
            Shuffleboard.getTab("main").add(key, (Sendable) value);
        }
    }

    public DoubleArrayTopic getRaspberryPiCameraPoseMeters(String cameraName){
        return rPiTable.getDoubleArrayTopic(cameraName);
    }

    public BooleanTopic getRaspberryPiValidReadingState(String cameraName){
        return rPiTable.getBooleanTopic(cameraName + "tagFound");
    }
    
    public NetworkTable getrPiTable(){
        return rPiTable;
    }

    public boolean pixyCamBroken() {
        return SmartDashboard.getBoolean("Pixy Cam Broken", false);
    }

    public boolean isFieldCentric() {
        return driveModeChooser.getSelected() == DriveMode.FIELD_CENTRIC;      
    }

    // Create enums for Dashboard elements/parts here
    public static enum DriveMode {
        FIELD_CENTRIC,
        ROBOT_CENTRIC;
    }
}
