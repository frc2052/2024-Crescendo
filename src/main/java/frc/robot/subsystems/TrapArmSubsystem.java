package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TrapArmSubsystem extends SubsystemBase{
    private final DigitalOutput trapArm;

    public TrapArmSubsystem() {
        trapArm = new DigitalOutput(Constants.Trap.TRAP_DIO_PIN);
    }

    public void open(){
        trapArm.set(true);
    }

    public void close(){
        trapArm.set(false);
    }
}
