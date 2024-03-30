package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.auto.AutoFactory.Auto;
import frc.robot.util.io.Dashboard;

/**
 * Subsystem to control the robot's LEDs, by determining what number should be encoded to DIO pins and
 * sent to the Arduino we used for controlling the patterns and colors
 */
public class LedSubsystem extends SubsystemBase {
    private static LedSubsystem INSTANCE;
    
    private final DigitalOutput codeChannel1, codeChannel2, codeChannel3, codeChannel4, codeChannel5;

    private LEDStatusMode currentStatusMode;

    private boolean disableLEDs;
    private boolean robotDisabled;

    private LedSubsystem() {
        // DIO outputs
        codeChannel1 = new DigitalOutput(Constants.LEDs.CHANNEL_1_PIN);
        codeChannel2 = new DigitalOutput(Constants.LEDs.CHANNEL_2_PIN);
        codeChannel3 = new DigitalOutput(Constants.LEDs.CHANNEL_3_PIN);
        codeChannel4 = new DigitalOutput(Constants.LEDs.CHANNEL_4_PIN);
        codeChannel5 = new DigitalOutput(Constants.LEDs.CHANNEL_5_PIN);
        robotDisabled = true;

        currentStatusMode = LEDStatusMode.OFF;

        // For manually inputing code to encode to DIO pins
        // SmartDashboard.putNumber("LED CODE", 0);
    }

    public static LedSubsystem getInstance() {  // Method to allow calling this class and getting the single instance from anywhere, creating the instance if the first time.
        if (INSTANCE == null) {
            INSTANCE = new LedSubsystem();
        }
        return INSTANCE;
    }

    public static enum LEDStatusMode {
        OFF(0), 
        DANGER(1),
        INTAKE(2),
        HAS_NOTE(3),
        AIMING(4),
        AIMING_ON_TARGET(5),
        SHOOTING(6),
        SHOOTING_ON_TARGET(7),
        DONE_SHOOTING(8),
        NO_AUTO(9),
        BLUE_AUTO(10),
        RED_AUTO(11),
        // TODO: make LED status for "intaking" blue pulse
        INTAKING(12);

        private final int code;

        private LEDStatusMode(int code) {
            this.code = code;
        }

        public int getPositionTicks() {
            return code;
        }
    }

    @Override
    public void periodic() {
        int code = 5;
        if(!disableLEDs) {

            // disabled 

            if (DriverStation.isDisabled()) {
                // If disabled, gets the alliance color from the driver station and pulses that. Only pulses color if connected to station or FMS, else pulses default disabled color (Firefl status mode)
                Auto selected = Dashboard.getInstance().getAuto();
                if (selected == Auto.NO_AUTO || selected == null){
                    currentStatusMode = LEDStatusMode.NO_AUTO;
                } else if (RobotState.getInstance().isRedAlliance()) {
                    currentStatusMode = LEDStatusMode.RED_AUTO;
                } else if (!RobotState.getInstance().isRedAlliance()) {
                   currentStatusMode = LEDStatusMode.BLUE_AUTO; 
                } else {
                    currentStatusMode = LEDStatusMode.OFF; // Reaches here if DriverStation.getAlliance returns Invalid, which just means it can't determine our alliance and we do cool default effect
                }

            // autonomous LED status modes

            } else if (DriverStation.isAutonomous()) {
                if(RobotState.getInstance().getNoteHeldDetected()){
                    currentStatusMode = LEDStatusMode.HAS_NOTE;
                } else if (RobotState.getInstance().getIsShamperAtGoalAngle()){
                    // currentStatusMode = LEDStatusMode.AIMING;
                } else {
                    if (RobotState.getInstance().isRedAlliance()) {
                        currentStatusMode = LEDStatusMode.RED_AUTO;
                    } else if (!RobotState.getInstance().isRedAlliance()) {
                        currentStatusMode = LEDStatusMode.BLUE_AUTO;
                    }
                }
            }

            // teleop LED status modes

            if (DriverStation.isTeleopEnabled()) {
                // shooting
                if(RobotState.getInstance().getShooting()){
                    if(!RobotState.getInstance().getNoteHeldDetected()){
                        currentStatusMode = LEDStatusMode.OFF;
                    } else if (RobotState.getInstance().getNoteHeldDetected() && RobotState.getInstance().getIsShamperAtGoalAngle() && RobotState.getInstance().getIsRotationOnTarget()){
                        currentStatusMode = LEDStatusMode.SHOOTING_ON_TARGET;
                    } else {
                        currentStatusMode = LEDStatusMode.SHOOTING;
                    } 
                }
                //  aimed
                else if (RobotState.getInstance().getNoteHeldDetected() && RobotState.getInstance().getIsShamperAtGoalAngle() && RobotState.getInstance().getIsRotationOnTarget()){
                    currentStatusMode = LEDStatusMode.AIMING_ON_TARGET;
                } 
                // aiming
                else if (RobotState.getInstance().getIsVerticalAiming() || RobotState.getInstance().getIsHorizontalAiming())
                {
                    currentStatusMode = LEDStatusMode.AIMING;
                } 
                else {
                    if(RobotState.getInstance().getNoteHeldDetected()){
                        currentStatusMode = LEDStatusMode.HAS_NOTE;
                    } else if (RobotState.getInstance().getIsIntaking()){
                        currentStatusMode = LEDStatusMode.INTAKING;
                    } else {
                        currentStatusMode = LEDStatusMode.OFF;
                    }
                }
            }

            code = currentStatusMode.code;
        } else {
            //LEDs are disabled
            code = 5;
        }

        // Code for encoding the code to binary on the digitalOutput pins
        Dashboard.getInstance().putData("Sending LED Code", code);
        codeChannel1.set((code & 1) > 0);   // 2^0
        codeChannel2.set((code & 2) > 0);   // 2^1
        codeChannel3.set((code & 4) > 0);   // 2^2
        codeChannel4.set((code & 8) > 0);   // 2^3
        codeChannel5.set((code & 16) > 0);  // 2^4
    }

    public void setLEDStatusMode(LEDStatusMode statusMode) {
        if (!disableLEDs) {
            currentStatusMode = statusMode;
        }
    }

    public LEDStatusMode getLEDStatusMode(){
        return currentStatusMode;
    }

    public void clearStatusMode() {
        currentStatusMode = LEDStatusMode.OFF;
    }

    // Disables LEDs (turns them off)
    public void disableLEDs() {
        disableLEDs = true;
    }

    // Enables LEDs (turns them on)
    public void enableLEDs() {
        disableLEDs = false;
    }

    public boolean getRobotDisabled(){
        return robotDisabled;
    }
}