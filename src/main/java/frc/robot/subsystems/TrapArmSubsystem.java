package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TrapArmSubsystem extends SubsystemBase{
    private final Relay trapArm;

    public TrapArmSubsystem() {
        trapArm = new Relay(Constants.Trap.TRAP_RELAY_PIN);
        trapArm.set(Relay.Value.kOff);
    }

    public void open(){
        trapArm.set(Relay.Value.kForward);
    }

    public void close(){
        trapArm.set(Relay.Value.kOff);
    }
}
