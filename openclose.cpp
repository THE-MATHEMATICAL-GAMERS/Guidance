#include <iostream>
#include <iomanip>
#include <tuple>
#include <string>
#include <sstream>
#include <stdlib.h>
#include <thread>
#include <chrono> 
#include <cmath>
#include <krpc.hpp>
#include <krpc/services/space_center.hpp>
#include "closedloopmath.hpp"

double fuelinstage_s(double mass, double dry) { return mass - dry; }
 
void guide(std::string ipaddr, double degrees, double hT)
{
    auto conn = krpc::connect("Guidance program", ipaddr);
    krpc::services::SpaceCenter sc(&conn);
    auto vessel = sc.active_vessel();

    // surface vessel velocity reference frame
    auto surf_ref_frame = vessel.orbit().body().reference_frame();
    auto controller = vessel.auto_pilot();
    auto parts = vessel.parts();

    // seting up stream
    auto vspeed = vessel.flight(surf_ref_frame).vertical_speed_stream();
    auto dir = vessel.flight(vessel.surface_velocity_reference_frame()).direction_stream();
    auto srbfuel = vessel.parts().with_tag("srb")[0].resources().amount_stream("HTPB");
    std::cout << vessel.flight().longitude() << " " << vessel.flight().latitude() << std::endl;

    controller.engage();
    controller.set_reference_frame(vessel.surface_reference_frame());
    controller.target_pitch_and_heading(90.0f, 90.0f);
    
    vessel.control().set_throttle(1);
    while(vspeed() < 50.0f);
    std::cout << "Turning to (90.0 - " << degrees << ", 90.0)." << std::endl;
    while(vspeed() >= 50.0f && vspeed() <= 100.0f)
    {
        controller.target_pitch_and_heading(float(90.0 - (vspeed() - 50.0)/50.0 * degrees), 90.0f);
    }

    std::cout << "Waiting for surface prograde to cross..." << std::endl;
    bool point_to_prograde = false;
    do
    {
        if(std::get<1>(dir()) > 0.999f)
        {
            point_to_prograde = true;
        }

    } while(!point_to_prograde);

    controller.disengage();
    controller.set_reference_frame((vessel.surface_velocity_reference_frame()));
    controller.engage();

    std::cout << "Pointing to surface prograde!" << std::endl;
    while(srbfuel() >= 0.001f)
    {
        controller.set_target_direction(std::make_tuple(0.0, 1.0, 0.0));
    };

    std::cout << "Open Loop guidance is over!" << std::endl;    
    std::cout << "Start Close loop guidance!" << std::endl;

    // Some constants for our vessel
    M2 = 40597.9; //kg
    I2 = 449.7; //s
    F2 = 203600; //N 
    stage = 0;
    double M1_dry = 64050.5; // dry mass to calculate fuelin stage
    
    F = vessel.max_thrust();

    auto vy_s = vessel.flight(surf_ref_frame).vertical_speed_stream();
    auto vx_s = vessel.flight(surf_ref_frame).horizontal_speed_stream();
    auto H_s = vessel.flight().mean_altitude_stream();
    auto Mass_s = vessel.mass_stream();
    auto F_s = vessel.thrust_stream();
    auto I_s = vessel.specific_impulse_stream();
    auto boostercheck = vessel.parts().with_tag("booster")[0].resources().amount_stream("LqdOxygen");
    auto lg = vessel.flight().longitude_stream();
    /***
    LIST: 
    - ElectricCharge
    - HTPB
    - LqdOxygen
    - Kerosene
    ***/

    std::cout << "Starting Close loop guidance...." << std::endl;
    controller.set_reference_frame(vessel.surface_reference_frame());

    do
    {
        Mass = Mass_s();
        vy = vy_s();
        vx = vx_s();
        H = H_s();
        fuelinStage = fuelinstage_s(Mass, M1_dry);
        F = F_s();
        I = I_s();

        if(F_s()/Mass > 44.145 && stage == 0)
        {
            std::cout << "Stage 1 activated!" << std::endl;
            stage = 1;
        }

        if(boostercheck() < 50.0f && stage != 2)
        {
            std::cout << "Seperation! Stage 2 activated!" << std::endl;
            std::this_thread::sleep_for (std::chrono::seconds(5));
            std::cout << "Restarting Guidance...." << std::endl;
            stage = 2;
        }

        calculate();
        auto pitch = 180.0*theta(0.0)/M_PI;
        auto head = acos(-0.47882041*sin((lg()+80.6)*M_PI/180.0))*180.0/M_PI;
        std::cout << " " << T << " " << pitch << " " << head << " " << controller.pitch_error() << " " << controller.heading_error() << std::endl;


        controller.target_pitch_and_heading(float(pitch), float(head));

    } while (T > 20.0);

    auto apo = vessel.orbit().apoapsis_altitude_stream();
    auto peri = vessel.orbit().periapsis_altitude_stream();

    std::cout << "Terminal Guidance time!" << std::endl;

    do
    {
        controller.target_pitch_and_heading(atan(-1*vy_s()/(vT - vx_s()))*180.0/M_PI, acos(-0.47882041*sin(lg()+80.6))*180.0/M_PI);
        
    } while (apo() + peri() < 600000);
    
    controller.disengage();
    vessel.control().set_sas(true);
    vessel.control().set_sas_mode(krpc::services::SpaceCenter::SASMode::prograde);
    vessel.control().set_throttle(0);

    std::cout << "The End! " << apo() << ", " << peri() << std::endl;
}

// inline argument as the ip to the krpc server
int main(int argv, char** argc)
{
    std::cout.flush();
    std::string input1 = argc[2];
    std::string input2 = argc[3];
    std::stringstream ss1 = std::stringstream(input1);
    std::stringstream ss2 = std::stringstream(input2);

    std::string ip(argc[1]);
    ss1 >> hT;
    ss2 >> T;
    // Pitch provided as second argument 90 - degrees upto 100 m/s
    guide(ip, 15.0, hT);
    
    return 0;
}