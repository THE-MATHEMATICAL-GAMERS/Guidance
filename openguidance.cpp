#include <iostream>
#include <iomanip>
#include <tuple>
#include <string>
#include <math.h>
#include <stdlib.h>
#include <krpc.hpp>
#include <krpc/services/space_center.hpp>
 
void openloop(std::string ipaddr, double degrees)
{
    auto conn = krpc::connect("Open loop guidance program", ipaddr);
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

    controller.disengage();

    vessel.control().set_sas(true);

    std::cout << "Open Loop guidance is over!" << std::endl;
}

// inline argument as the ip to the krpc server
int main(int argv, char** argc)
{
    std::cout.flush();
    std::string ip(argc[1]);
    // Pitch provided as second argument 90 - degrees upto 100 m/s
    openloop(ip, 10.0);

    std::cout << "Start Close loop guidance!" << std::endl;
    
    return 0;
}