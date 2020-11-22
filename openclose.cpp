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

bool end = false;
double fuelinstage_s(double mass, double dry) { return mass - dry; }
 
void oguide(std::string ipaddr, double degrees)
{
    auto conn = krpc::connect("Open Guidance program", ipaddr);
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
    std::this_thread::sleep_for (std::chrono::seconds(1));
    vessel.control().set_throttle(0);
    while(vspeed() < 50.0f);
    std::cout << "Turning to (90.0 - " << degrees << ", 90.0)." << std::endl;
    while(vspeed() >= 25.0f && vspeed() <= 125.0f)
    {
        controller.target_pitch_and_heading(float(90.0 - (vspeed() - 25.0)/100.0 * degrees), 90.0f);
    }

    std::cout << "Waiting for surface prograde to cross..." << std::endl;
    bool point_to_prograde = false;
    do
    {
        if(std::get<1>(dir()) > 0.9999f)
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

    controller.disengage();
}

void cguide(std::string ipaddr)
{
    auto conn = krpc::connect("Closed Guidance program", ipaddr);
    krpc::services::SpaceCenter sc(&conn);
    auto vessel = sc.active_vessel();

    // orbit vessel velocity reference frame
    auto surf_ref_frame = vessel.orbit().body().reference_frame();
    auto orbit_ref_frame = vessel.orbit().body().non_rotating_reference_frame();
    auto controller = vessel.auto_pilot();

    // Some constants for our vessel
    M2 = 40292; //kg
    I2 = 449.7; //s
    F2 = 203600; //N 
    double M1_dry = 63744.6; // dry mass to calculate fuelin stage
    
    F = vessel.max_thrust();

    auto vy_s = vessel.flight(orbit_ref_frame).vertical_speed_stream();
    auto vx_s = vessel.flight(orbit_ref_frame).horizontal_speed_stream();
    auto H_s = vessel.flight().mean_altitude_stream();
    auto Mass_s = vessel.mass_stream();
    auto F_s = vessel.thrust_stream();
    auto I_s = vessel.specific_impulse_stream();
    auto lg = vessel.flight().longitude_stream();
    /***
    LIST: 
    - ElectricCharge
    - HTPB
    - LqdOxygen
    - Kerosene
    ***/

    std::cout << "Starting Close loop guidance...." << std::endl;
    controller.engage();
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
        auto prev_A = A;
        auto prev_B = B;
        auto prev_T = T;

        if(F_s()/Mass > 44.145 && stage == 0)
        {
            std::cout << "Stage 1 activated!" << std::endl;
            stage = 1;
        }

        if(stage != 2 && vessel.parts().with_tag("booster").size() <= 0)
        {
            std::cout << "Seperation! Stage 2 activated!" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(2500));
            std::cout << "Restarting Guidance...." << std::endl;
            stage = 2;
        }

        if(F == 0.0)
        {
            vessel.control().set_throttle(1);
        }

        calculate();
        if (std::isnan(theta(0.0)) || std::isnan(T))
        {
            A = prev_A;
            B = prev_B;
            T = prev_T;
            std::cout << "NaN occured. Performing again..." << std::endl;
            continue;
        }

        auto pitch = 180.0*theta(0.0)/M_PI;
        auto head = acos(-0.47882041*sin((lg()+80.6)*M_PI/180.0))*180.0/M_PI;
        std::cout << " " << T << " " << pitch << std::endl;


        controller.target_pitch_and_heading(float(pitch), float(head));

    } while (T > 20.0);

    auto apo = vessel.orbit().apoapsis_altitude_stream();
    auto peri = vessel.orbit().periapsis_altitude_stream();

    std::cout << "Terminal Guidance time!" << std::endl;

    do
    {
        auto theta = atan(-1 * vy_s() / (vT - vx_s())) * 180.0 / M_PI;
        auto throttle = (-1 * Mass_s() * vy_s() * vy_s() / (2 * (hT - H_s()) * sin(theta / 180.0 * M_PI)) / vessel.max_thrust() - 0.3) / 0.7;
        if (throttle >= 1)
        {
            throttle = 1.0f;
        }
        else if (throttle <= 0.001f)
        {
            throttle = 0.001f;
        }
        controller.target_pitch_and_heading(theta, acos(-0.47882041*sin((lg()+80.6)*M_PI / 180.0))*180.0/M_PI);
        vessel.control().set_throttle(throttle);
        std::cout << " " << 2 * (hT - H_s()) /vy_s() << " " << theta << std::endl;

    } while (apo() + peri() < 2*hT - 2000);

    controller.disengage();
    vessel.control().set_throttle(0);
    vessel.control().set_rcs(true);

    vessel.control().set_forward(1);
    do
    {
        std::cout << " " << 2 * (hT - H_s()) / vy_s() << " " << theta << std::endl;

    } while (apo() + peri() < 2*hT);
    vessel.control().set_forward(0);
    
    vessel.control().set_sas(true);
    vessel.control().set_sas_mode(krpc::services::SpaceCenter::SASMode::prograde);

    std::cout << "The End! " << apo() << ", " << peri() << std::endl;

    end = true;
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
    oguide(ip, 15.0);

    std::cout << "Connecting back...." << std::endl;

    stage = 0;

    while (!end)
    {
        try
        {
            cguide(ip);
        }
        catch(krpc::RPCError error)
        {
            std::cout << "ERROR HAPPENED CONNECTING.... RECONNECTING...." << std::endl;
            continue;
        }
    }
    
    return 0;
}