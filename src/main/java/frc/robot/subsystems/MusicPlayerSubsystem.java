package frc.robot.subsystems;



import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MusicPlayerSubsystem extends SubsystemBase{

    private final Orchestra musicPlayer;
    private final TalonFX[] instrumentList;

    public MusicPlayerSubsystem() {
        musicPlayer = new Orchestra();
        instrumentList = Constants.MusicPlayer.INSTRUMENT_TALONFX_PORT_LIST;

        for (int i = 0; i < instrumentList.length; i++) {
            musicPlayer.addInstrument(instrumentList[i]);
        }
    }

    public void loadMusic(String filepath) {
        musicPlayer.loadMusic(filepath);
    }

    public void playLoadedTrack() {
        musicPlayer.play();
    }

    public void pauseMusic() {
        musicPlayer.pause();
    }

    public void stopMusic() {
        musicPlayer.stop();
    }

    public double getCurrentPlayTime() {
        return musicPlayer.getCurrentTime();
    }

    public boolean isPlayerPLaying() {
        return musicPlayer.isPlaying();
    }
}
