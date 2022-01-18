#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace std::experimental;

using std::string;
using std::cout;
using std::cin;


static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate}; // creating a stream which receives the stuff as a binary and immediately goes to the end (to be able to get the size of the stream easily)
    if( !is )
        return std::nullopt;
    
    auto size = is.tellg(); // here we are getting the size
    std::vector<std::byte> contents(size);    
    
    is.seekg(0); // going back to the start of the stream
    is.read((char*)contents.data(), size); // reading the content of the stream into the vector

    if( contents.empty() )
        return std::nullopt;
    return std::move(contents);
}

// Asks the user for input, checks the entry, and returns the entered float
// promptMessage: the message to display
// max: the  expected maximal value
// min: the expected minimal value
// returns: the float entered by the user
static float getUserInput(string promptMessage, float min, float max){
    bool entryAccepted = false;
    float entered;
    while(!entryAccepted){
        cout << promptMessage << "\n";
        cin >> entered;
        cout << "You have entered " << entered << "\n";
        if (entered < min){
            cout << "The number must be at least " << min << "\n";
            continue;
        }
        if (entered > max){
            cout << "The number must be at most " << max << "\n";
            continue;
        }

        entryAccepted = true;
    }

    return entered;
}

int main(int argc, const char **argv)
{    
    std::string osm_data_file = "";
    if( argc > 1 ) {
        for( int i = 1; i < argc; ++i )
            if( std::string_view{argv[i]} == "-f" && ++i < argc )
                osm_data_file = argv[i];
    }
    else {
        std::cout << "To specify a map file use the following format: " << std::endl;
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
        osm_data_file = "../map.osm";
    }
    
    std::vector<std::byte> osm_data;
 
    if( osm_data.empty() && !osm_data_file.empty() ) {
        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if( !data )
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }
    
    // TODO 1: Declare floats `start_x`, `start_y`, `end_x`, and `end_y` and get
    // user input for these values using std::cin. Pass the user input to the
    // RoutePlanner object below in place of 10, 10, 90, 90.

    float minX = 0;
    float maxX = 100;
    float minY = 0;
    float maxY = 100;

    float startX = getUserInput("Enter the x coordinate of the starting position.", minX, maxX);
    float startY = getUserInput("Enter the y coordinate of the starting position.", minY, maxY);

    float endX = getUserInput("Enter the x coordinate of the end position.", minX, maxX);
    float endY = getUserInput("Enter the y coordinate of the end position.", minY, maxY);

    // Build Model.
    RouteModel model{osm_data};

    // Create RoutePlanner object and perform A* search.
    RoutePlanner route_planner{model, startX, startY, endX, endY};
    route_planner.AStarSearch();

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

    // Render results of search.
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    display.begin_show();
}
