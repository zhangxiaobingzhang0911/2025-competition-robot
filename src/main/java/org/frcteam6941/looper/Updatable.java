package org.frcteam6941.looper;
 
// Interface for objects that need to be updated periodically in a loop
public interface Updatable {
    // Method to read input or state from sensors or other systems
    default void read(double time, double dt) {}
 
    // Method to update the object's internal state based on the read data and time step
    default void update(double time, double dt) {}
 
    // Method to write output or commands to actuators or other systems
    default void write(double time, double dt) {}
 
    // Method to provide telemetry data for debugging and monitoring purposes
    default void telemetry() {}
 
    // Method to start the object, typically initializing resources or enabling systems
    default void start() {}
 
    // Method to stop the object, typically freeing resources or disabling systems
    default void stop() {}
 
    // Method to simulate the read process, used for testing and development
    default void simulate(double time, double dt) {read(time, dt);}
}