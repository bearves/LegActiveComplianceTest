# Todolist

* Leg inverse kinematic
* Leg impedance controller
    * Input: The foot point at last moment; The force from the force sensor; -> The force w.r.t. the hip joint; -> 
             The virtual model of the impedance: xddot = (dF - Kdx - bdxdot)/M -> Integrator -> Position offset.
    * Output: Position offset.
* Leg Trajectory generator for walking
* Leg Trajectory generator for goto the start point from anywhere

Class LegKinematic: Inverse kinematic for any leg

Class LegImpedanceController

Class GaitTrajectoryGenerator

Class ModelExchanger: Map machine data to the model

