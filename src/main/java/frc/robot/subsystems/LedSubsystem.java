package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.states.RobotState;

public class LedSubsystem extends SubsystemBase {
     private static LedSubsystem INSTANCE;
    
    private final DigitalOutput codeChannel1, codeChannel2, codeChannel3, codeChannel4, codeChannel5;
    private LEDStatusMode currentStatusMode;

    private boolean disableLEDs;
    private boolean robotDisabled;
    


    private LedSubsystem() {
        // DIO outputs
        codeChannel1 = new DigitalOutput(Constants.LED.CHANNEL_1_PIN);
        codeChannel2 = new DigitalOutput(Constants.LED.CHANNEL_2_PIN);
        codeChannel3 = new DigitalOutput(Constants.LED.CHANNEL_3_PIN);
        codeChannel4 = new DigitalOutput(Constants.LED.CHANNEL_4_PIN);
        codeChannel5 = new DigitalOutput(Constants.LED.CHANNEL_5_PIN);
        
        robotDisabled = true;
        

        currentStatusMode = LEDStatusMode.OFF;

}

    public static LedSubsystem getInstance() { 
            if (INSTANCE == null) {
            INSTANCE = new LedSubsystem();
        }
        return INSTANCE;
    }


    public static enum LEDStatusMode {
        amp(0),
        coop(1),
        neutral(2),
        red(3),
        blue(4),
        OFF(5),
        rainbow(6),
        green(7),
        knightkrawler(8);


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
        int code = 0;
        if(!disableLEDs) {
            if (robotDisabled) {
                if  (RobotState.getInstance().isRedAlliance()) {
                    currentStatusMode = LEDStatusMode.red;
                } else if (!RobotState.getInstance().isRedAlliance()) {
                    currentStatusMode = LEDStatusMode.blue;
                } else {
                    currentStatusMode = LEDStatusMode.neutral;
                }
            }

            code = currentStatusMode.code;
        } else {
            code = 2;
        }

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

    public void robotDisabled(){
        robotDisabled = true;
    }

    public void robotEnabled(){
        robotDisabled = false;
    }

    public boolean getRobotDisabled(){
        return robotDisabled;
    }
}







