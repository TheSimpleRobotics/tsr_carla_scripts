/*
 * Author: Denis Tananaev
 * Date: 17.03.2019
 *
 *
 */

#include <carla/client/World.h>
#include <carla/client/Client.h>
#include <carla/rpc/WeatherParameters.h>
#include <string>
#include <pthread.h>
int main(){
 std::cout<<"This is minimal example of the c++ API for carla.  \
             The program creates the client and connects to it. \
             Afterwards it changes the weather of the simulator \
             to heavy rain. "<<"\n";

 // The host and port for carla 
 std::string host = "127.0.0.1";
 uint16_t port = 2000;	

 // Create carla client 
 carla::client::Client MyClient(host, port);

 // Add timeout for 1 second to try to connect to server
 size_t t = 1;
 auto timeout=carla::time_duration::seconds(t);
 MyClient.SetTimeout(timeout);

 // Retrieve the world
 carla::client::World MyWorld=MyClient.GetWorld();
 // Weather parameters
 float cloudyness = 100.0f;
 float precipitation = 100.0f;
 float precipitation_deposits = 100.0f;
 float wind_intensity = 100.0f;
 float sun_azimuth_angle = 100.5f;
 float sun_altitude_angle = 100.5f;
 // It is possible define weather like this
 carla::rpc::WeatherParameters weather= carla::rpc::WeatherParameters(cloudyness,
 	                                                                  precipitation,
 	                                                                  precipitation_deposits,
 	                                                                  wind_intensity,
 	                                                                  sun_azimuth_angle,
 	                                                                  sun_altitude_angle);
 // Or just use predefined settings like HardRainNoon
 MyWorld.SetWeather(carla::rpc::WeatherParameters::HardRainNoon);

}