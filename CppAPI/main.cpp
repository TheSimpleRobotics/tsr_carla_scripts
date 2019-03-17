/*
 * Author: Denis Tananaev
 * Date: 17.03.2019
 *
 *
 */

#include <carla/client/World.h>
#include <carla/client/Client.h>
#include <string>
#include <pthread.h>
int main(){


 // The host and port for carla 
 std::string host = "127.0.0.1";
 uint16_t port = 2000;	

 // Create carla client 
 carla::client::Client MyClient(host, port);

 // Add timeout
 size_t t = 10;
 auto timeout=carla::time_duration::seconds(t);
 MyClient.SetTimeout(timeout);


 std::cout<<"I made something that is working!!!!"<<"\n";

}