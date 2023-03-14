#include <cstddef>
#include <chrono>
#include <functional>
#include <iomanip>
#include <mutex>
#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <array>
#include <string>
#include <thread>
#include "model.h"
#include <unistd.h>
#include "render.h"
#include "draw_image.h"
#include <io2d.h>
#undef None // X11 from io2d sets #define None 0
#undef Success
#undef Status
#include "dashIceoryx/create_subscriber.hpp"
#include "dashIceoryx/utils.hpp"
#include "dashTypes/sensor.hpp"
#include "dashSerial/sensor.hpp"

namespace io2d = std::experimental::io2d;

namespace OSM_PathPlotter
{
std::vector<Model::Node> input_route;
std::vector<Model::Node> non_precise_points;


class ReadfromFile
{

  public:
    
    
    int FindCounter(const std::string& data)
    {
        std::string str = "Counter: ";
        auto position = data.find(str);
        if (position <= data.size())
        {
            return std::stoi(data.substr(position+str.size()));
        }
        return 0;
    }

    float FindPrecision(const std::string& data)
    {
        std::string str = "GPS precision: ";
        auto position = data.find(str);
        if (position <= data.size())
        {
            return std::stof(data.substr(position+str.size()));
        }
        return 0;
    }

    void FindCovariance(const std::string& data, std::array<float,2>& covariance)
    {
        std::string str = "Xcovariance: ";
        auto position = data.find(str);
        if (position <= data.size())
        {
            covariance[0] = std::stod(data.substr(position+str.size(), 10));
        }
        str = "Ycovariance: ";
        position = data.find(str);
        if (position <= data.size())
        {
            covariance[1] = std::stod(data.substr(position+str.size(), 10));
        } 
        std::cout << std::setprecision(8) << std::fixed << covariance[0] << ", " << covariance[1] << std::endl;
        return;
    }
    Model::Node FindPoint(const std::string& data, Model& model)
    {
        Model::Node current_node;
        std::string str = "Latitude: ";
        auto position = data.find(str);
        double lat = 200;
        if (position <= data.size())
        {  
            lat = std::stod(data.substr(position+str.size())); 
        }
        str = "Longitude: ";
        position = data.find(str);
        double lon = 200;
        if (position <= data.size())
        {
            lon = std::stod(data.substr(position+str.size()));
        }
        if (lat != 200 && lon != 200)
        {
            current_node = model.AdjustCoordinates(lat, lon);    
        }
        return current_node;
    }

};

void ExportImage(Model model)
{
    DrawImage draw{model};
    auto img = io2d::image_surface{io2d::format::argb32, 400, 400};
    draw.ExportImage(img, input_route, non_precise_points);
    img.save("map.png", io2d::image_file_format::png);
}

void AddInputFromIPC(Model model)
{
    dashIPC::IoxRuntimeManager::Init("GPS Subscriber");
    std::mutex receivedDataMtx;
    std::vector<dashTypes::GPSData> receivedData;
    std::function<void(dashTypes::GPSData&&)>recvFn = [&] (dashTypes::GPSData&& input_data){
        std::this_thread::sleep_for(std::chrono::seconds(2));
        std::lock_guard lock(receivedDataMtx);
        receivedData.push_back(input_data);
    };
    auto m_gpsSubscriber = dashIPC::CreateSubscriber("Robot", "Sensor", "GPS", recvFn, iox::popo::SubscriberOptions{.queueCapacity = 100U, .queueFullPolicy = iox::popo::QueueFullPolicy::BLOCK_PRODUCER});
    for (auto & i : receivedData)
    {
        auto x_cov = static_cast<float>(i.positional_covariance_NED(0,0));
        auto y_cov = static_cast<float>(i.positional_covariance_NED(1,1));
        double lat = i.latitude;
        double lon = i.longitude;
        if (x_cov<=15.0f && y_cov<=15.0f)
        {
            input_route.emplace_back(model.AdjustCoordinates(lat, lon));
        }
        else {
            non_precise_points.emplace_back(model.AdjustCoordinates(lat, lon));
        }
    }
    return;
}

void AddInputfromFile(Model model)
{
    ReadfromFile readfile;
    std::string path = "GPSinput.txt";
    std::ifstream GPS_input;
    GPS_input.open(path);
    std::string data;
    std::array<float,2> covariance;
    if (GPS_input.is_open())
    {
        while (std::getline(GPS_input, data))
        {
            readfile.FindCovariance(data, covariance);
            // if (readfile.FindPrecision(data) <= 3.0f && readfile.FindPrecision(data) > 0.0f)
            if (covariance[0] <= 12.0f && covariance[1] <= 12.0f)
            {
                input_route.emplace_back(readfile.FindPoint(data, model));
            }
            else
            {
                non_precise_points.emplace_back(readfile.FindPoint(data, model));
            }
            ExportImage(model);
            // sleep(1);
        }
    }
    GPS_input.close();
}

void AddInputCoordinates(Model model)
{
    Model::Node current_node;
    // points in lat, lon format
    // if not needed, change datatype from double to float
    std::array<std::array<double,2>,9> points = {{{{1.2957234,103.7964777}}, {{1.2952231, 103.7971796}}, {{1.2961383, 103.7942698}}, {{1.2946667, 103.7963543}}, {{1.2955908, 103.7960161}}, {{1.2942886, 103.7972996}}, {{1.2956843, 103.7951322}},{{1.2956286, 103.7968542}},{{1.2953962, 103.7961267}}}};
    
    std::array<std::array<double,2>,3> np_points = {{{{1.2956720, 103.7982637}}, {{1.2946837, 103.79514332}}, {{1.2957234, 103.7968542}}}};
    for (size_t i=0; i<points.size(); i++)
    {
        current_node = model.AdjustCoordinates(points[i][0], points[i][1]);
        input_route.emplace_back(current_node);
        if (i<np_points.size())
        {
            current_node = model.AdjustCoordinates(np_points[i][0], np_points[i][1]);
            non_precise_points.emplace_back(current_node);
        }
        ExportImage(model);
        sleep(1);
    }
}

void RunRender(Model model)
{
    Render render{model};
    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface)
    {    
        render.Display(surface, input_route, non_precise_points);
    });
    display.begin_show();
}

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if (!is) 
    {
        return std::nullopt;
    }
    auto size = is.tellg();
    std::vector<std::byte> contents(size);    
    
    is.seekg(0);
    is.read((char*)contents.data(), size);

    if (contents.empty()) 
    {
        return std::nullopt;
    }
    return std::move(contents);
}
} // namespace OSM_PathPlotter



using namespace OSM_PathPlotter;

int main(int argc, const char** argv)
{    
    std::string osm_data_file = "";
    if (argc == 2)
    {
        osm_data_file = argv[1];
    }
    else 
    {
        std::cout << "Wrong input argument, please follow: [executable] [filename.osm]" << std::endl;
        return 0;
    }
    
    std::vector<std::byte> osm_data;
 
    if (osm_data.empty() && !osm_data_file.empty()) 
    {
        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if (!data) 
        {
            std::cout << "Failed to read." << std::endl;
        }
        else 
        {
            osm_data = std::move(*data);
        }
  	}
    
    // Build Model.
    Model model{osm_data};
    
    // Test code
    // AddInputCoordinates(model);
    // AddInputfromFile(model); 
    // AddInputFromIPC(model);
    // RunRender(model);
    
    std::thread first(AddInputfromFile, model);
    // std::thread first(AddInputCoordinates, model);
    // std::thread first(AddInputFromIPC, model);
    std::thread second(RunRender, model);

    first.join();
    second.join();
    
    return 0;    
}



