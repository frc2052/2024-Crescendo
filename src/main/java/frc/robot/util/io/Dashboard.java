package frc.robot.util.io;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Dashboard {
    private static Dashboard INSTANCE;

    private final SendableChooser<DriveMode> driveModeChooser;

    public static Dashboard getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Dashboard();
        }

        return INSTANCE;
    }

    private Dashboard() {
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

    public boolean isFieldCentric() {
        return driveModeChooser.getSelected() == DriveMode.FIELD_CENTRIC;      
    }

    // Enums for Dashboard elements:
    public static enum DriveMode {
        FIELD_CENTRIC,
        ROBOT_CENTRIC;
    }
}