#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define LENGTH 300
#define TIME_STEP 0.1

struct State_Feedback_Control
{
    const float k1;             // k1 gain
    const float k2;             // k2 gain
    const float k3;             // k3 gain
    const float kr;             // Setpoint gain constant
    const float T_C;            // Time constant for derivative filtering
    const float T;              // Time step
    const float max;            // Max command
    const float min;            // Min command
    const float max_rate;       // Max rate of change of the command
    float integral;             // Integral term
    float z_prev;               // Previous value of z
    float z_deriv_prev;         // Previous derivative of z
    float command_sat_prev;     // Previous saturated command
    float command_prev;         // Previous command
};

struct Object
{
    const float m;              // Mass of the object
    const float k;              // Damping constant
    const float F_max;          // Max force applied to the object
    const float F_min;          // Min force applied to the object
    const float T;              // Time step
    float v;                    // Velocity of the object
    float z;                    // Position of the object
};

float State_Feedback_Control_Step(struct State_Feedback_Control *state_feedback_control, float measurement, float setpoint)
{
    /* This function implements a state feedback controller.
     *
     * Inputs:
     *   measurement: current measurement of the process variable
     *   setpoint: desired value of the process variable
     *   state_feedback_control: a pointer to a state feedback controller struct containing the controller parameters
     *
     * Returns:
     *   command_sat: the control output of the state feedback controller (saturated based on max. min, max_rate)
     */

    float err;
    float command;
    float command_sat;
    float z_deriv_filt;

    /* Error calculation */
    err = setpoint - measurement;

    /* Integral term calculation */
    state_feedback_control->integral += state_feedback_control->k3*err*state_feedback_control->T;
    
    /* Derivative of z using filtered derivative method */
    z_deriv_filt = (measurement - state_feedback_control->z_prev + state_feedback_control->T_C*state_feedback_control->z_deriv_prev)/(state_feedback_control->T + state_feedback_control->T_C);
    state_feedback_control->z_prev = measurement;
    state_feedback_control->z_deriv_prev = z_deriv_filt;

    /* Calculate command */
    command = state_feedback_control->kr*setpoint - state_feedback_control->k1*measurement - state_feedback_control->k2*z_deriv_filt + state_feedback_control->integral;

    /* Remember command at previous step */
    state_feedback_control->command_prev = command;

    /* Saturate command */
    if (command > state_feedback_control->max)
    {
        command_sat = state_feedback_control->max;
    }
    else if (command < state_feedback_control->min)
    {
        command_sat = state_feedback_control->min;
    }
    else
    {
        command_sat = command;
    }

    /* Apply rate limiter */
    if (command_sat > state_feedback_control->command_sat_prev + state_feedback_control->max_rate*state_feedback_control->T)
    {
        command_sat = state_feedback_control->command_sat_prev + state_feedback_control->max_rate*state_feedback_control->T;
    }
    else if (command_sat < state_feedback_control->command_sat_prev - state_feedback_control->max_rate*state_feedback_control->T)
    {
        command_sat = state_feedback_control->command_sat_prev - state_feedback_control->max_rate*state_feedback_control->T;
    }
    else
    {
        /* No action */
    }

    /* Remember saturated command at previous step */
    state_feedback_control->command_sat_prev = command_sat;

    return command_sat;
}

float Object_Step(struct Object *obj, float F, float Fd){

    /* This function updates the position of an object in 1D based on the applied force F and
     * the object's mass, viscous damping coefficient k, max/min forces, disturbance force Fd, and time step T.
     *
     * Inputs:
     *   F: the force applied to the object
     *   Fd: the disturbance force
     *   obj: a pointer to an object struct containing its properties (mass, damping, etc.)
     *
     * Returns:
     *   z: the position of the object in meters
     */

    /* Declare variables for the derivative dv/dt and the saturated force command */
    float dv_dt;
    float F_sat;

    /* Apply saturation to the input force */
    if (F > obj->F_max)
    {
        F_sat = obj->F_max;
    }
    else if (F < obj->F_min)
    {
        F_sat = obj->F_min;
    }
    else
    {
        F_sat = F;
    }

    /* Calculate the derivative dv/dt using the input force and the object's velocity and properties */
    dv_dt = (F_sat - obj->k*obj->v - Fd)/obj->m;

    /* Update the velocity and position of the object by integrating the derivative using the time step T */
    obj->v += dv_dt*obj->T;
    obj->z += obj->v*obj->T;

    /* Return the updated position of the object */
    return obj->z;
}


int main()
{
    // Current simulation time
    float t = 0;

    // Iteration counter
    int i = 0;

    // Setpoint and output of the first control loop
    float command = 0;
    float stp = 100;
    float z = 0;

    // State feedback controller initialisation
    struct State_Feedback_Control state_feedback_control = {4.8, 11.5, 0.64, 1.5, 0, TIME_STEP, 300, -300, 100, 0, 0, 0, 0, 0};

    // Object parameters for the first control loop
    struct Object obj = {10, 0.5, 300, -300, TIME_STEP, 0, 0};

    // Open a file for logging simulation data
    FILE *file = fopen("data.txt", "w");

    /* Implement iteration using a while loop */
    while(i < LENGTH)
    {
        
        // Execute the first control loop
        command = State_Feedback_Control_Step(&state_feedback_control, z, stp);
        z = Object_Step(&obj, command, 100);

        // Log the current time and control loop values to the file
        fprintf(file, "%f %f %f %f\n", t, command, z, stp);

        // Increment the time and iteration counter
        t = t + TIME_STEP;
        i = i + 1;
    }

    // Close the file and exit the program
    fclose(file);
    exit(0);
}
