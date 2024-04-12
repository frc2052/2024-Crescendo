package frc.robot.auto;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.util.io.Dashboard;

/**
 * Responsible for selecting, compiling, and recompiling autos before the start of a match.
 */
public class AutoFactory {
    private final Supplier<Auto> autoSupplier;
    private Auto currentAuto;
    private Command compiledAuto;


    public AutoFactory(
        Supplier<Auto> autoSupplier
    ) {
        this.autoSupplier = autoSupplier;
    }


    public Command getCompiledAuto() {
        return compiledAuto;
    }

    public boolean recompileNeeded() {
        return autoSupplier.get() != currentAuto;
    }

    public void recompile() {
        Dashboard.getInstance().putData(Constants.Dashboard.AUTO_COMPILED_KEY, false);

        currentAuto = autoSupplier.get();
        if (currentAuto == null) {
            currentAuto = Auto.NO_AUTO;
        }
        
        if (currentAuto.getAuto() != null) {
            compiledAuto = AutoBuilder.buildAuto(currentAuto.getAuto().getName());
            System.out.println("Current Auto: " + currentAuto.getAuto().getName());

            // don't need to initialize the command according to bryan
            // if (compiledAuto != null) {
            //     compiledAuto.initialize();
            // }
        }

        Dashboard.getInstance().putData(Constants.Dashboard.AUTO_COMPILED_KEY, true);
    }


    /*
     * This is where all our autos are. Need to use this as our list because when deploying to robot old autos don't get 
     * cleared from the RoboRIO and may show up with built in PathPlannerLib Auto Selector
     */
    public static enum Auto {
        NO_AUTO(null),
        AUTO_C2617(new PathPlannerAuto("Auto-C2617")),
        AUTO_A387(new PathPlannerAuto("Auto-A387")),
        AUTO_S54(new PathPlannerAuto("Auto-S54")),
        AUTO_S45(new PathPlannerAuto("Auto-S45")),
        AUTO_C123(new PathPlannerAuto("Auto-C123")),
        AUTO_C12387(new PathPlannerAuto("Auto-C12387")),
        ALIGNMENT_4(new PathPlannerAuto("Alignment 45"));
        // alignment_test_auto(new PathPlannerAuto("alignment test auto"));

        private final PathPlannerAuto auto;

        private Auto(PathPlannerAuto auto) {
            this.auto = auto;
        }

        public PathPlannerAuto getAuto(){
            return auto;
        }
    }
}



