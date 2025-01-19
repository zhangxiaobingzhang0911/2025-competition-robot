package org.frcteam6941.looper;
 
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
 
import frc.robot.RobotConstants;
 
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
 
public class UpdateManager {
    // List to hold all updatables that need to be managed
    public final List<Updatable> updatables = new ArrayList<>();
    // Lock object to synchronize access to the updatables list during updates
    private final Object taskRunningLock_ = new Object();
    // Timestamp of the last update loop execution
    private double lastTimestamp = 0.0;
    // Runnable task for simulation mode updates
    private final Runnable simulationRunnable = () -> {
        synchronized (taskRunningLock_) {
            // Iterate over each updatable and perform simulation, update, write, and telemetry tasks
            updatables.forEach(s -> {
                double fpgaTime = Timer.getFPGATimestamp();
                final double timestamp = fpgaTime != 0.0 ? fpgaTime : lastTimestamp;
                final double dt = timestamp - lastTimestamp > 10e-5 ? timestamp - lastTimestamp
                        : RobotConstants.LOOPER_DT;
                lastTimestamp = timestamp;
                s.simulate(timestamp, dt);
                s.update(timestamp, dt);
                s.write(timestamp, dt);
                s.telemetry();
            });
        }
    };
    // Notifier to manage the periodic execution of the simulation update task
    private final Notifier updaterSimulationThread = new Notifier(simulationRunnable);
    // Runnable task for enabled mode updates
    private final Runnable enableRunnable = new Runnable() {
        @Override
        public void run() {
            synchronized (taskRunningLock_) {
                // Iterate over each updatable and perform read, update, write, and telemetry tasks
                updatables.forEach(s -> {
                    double fpgaTime = Timer.getFPGATimestamp();
                    final double timestamp = fpgaTime > 10e-5 ? fpgaTime : lastTimestamp;
                    final double dt = timestamp - lastTimestamp > 10e-5 ? timestamp - lastTimestamp
                            : RobotConstants.LOOPER_DT;
                    lastTimestamp = timestamp;
                    s.read(timestamp, dt);
                    s.update(timestamp, dt);
                    s.write(timestamp, dt);
                    s.telemetry();
                });
            }
        }
    };
    // Notifier to manage the periodic execution of the enabled update task
    private final Notifier updaterEnableThread = new Notifier(enableRunnable);
 
    // Constructor to initialize the update manager with an array of updatables
    public UpdateManager(Updatable... updatables) {
        this(Arrays.asList(updatables));
    }
 
    // Constructor to initialize the update manager with a list of updatables
    public UpdateManager(List<Updatable> updatables) {
        this.updatables.addAll(updatables);
    }
 
    // Method to start the periodic update loop for enabled mode
    public void startEnableLoop(double period) {
        updaterEnableThread.startPeriodic(period);
    }
 
    // Method to run a single update cycle for enabled mode
    public void runEnableSingle() {
        enableRunnable.run();
    }
 
    // Method to stop the periodic update loop for enabled mode
    public void stopEnableLoop() {
        updaterEnableThread.stop();
    }
 
    // Method to start the periodic update loop for simulation mode
    public void startSimulateLoop(double period) {
        updaterSimulationThread.startPeriodic(period);
    }
 
    // Method to run a single update cycle for simulation mode
    public void runSimulateSingle() {
        simulationRunnable.run();
    }
 
    // Method to stop the periodic update loop for simulation mode
    public void stopSimulateLoop() {
        updaterSimulationThread.stop();
    }
 
    // Method to invoke the start method on all registered updatables
    public void invokeStart() {
        updatables.forEach(Updatable::start);
    }
 
    // Method to invoke the stop method on all registered updatables
    public void invokeStop() {
        updatables.forEach(Updatable::stop);
    }
 
    // Method to register all subsystems among the updatables with the command scheduler
    public void registerAll() {
        updatables.forEach((Updatable u) -> {
            if (u instanceof Subsystem adapted) {
                CommandScheduler.getInstance().registerSubsystem(adapted);
            }
        });
    }
}