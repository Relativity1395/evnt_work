#include <../include/OpticalFlow.hpp>
#include <iostream>

// this is definitely broken now sorry bro

int main(){
    std::cout <<"Window 1 Test Started\n";
    Event event1;
    event1.position = Eigen::Vector2d(10.0,10.0);
    event1.timestamp = .7;
    Eigen::Vector2d Ui = Eigen::Vector2d(2,1);
    double Ti = 1.0;
    Event event2;
    event2.position = Eigen::Vector2d(10.2,10.1);
    event2.timestamp = .8;
    Event event3;
    event3.position = Eigen::Vector2d(10.4,10.2);
    event3.timestamp = .9;
    std::vector<Event> Events;
    Events.push_back(event1);
    Events.push_back(event2);
    Events.push_back(event3);
    Eigen::Vector2d startPosition(0,0);
    Feature feature(startPosition, Events, Events, Ui, 0.0);
    feature.propagatePreviousEvents(Ti);
    const std::vector<Eigen::Vector2d>& result = feature.getLandmark();
    std::cout << result[0].x() << "," << result[0].y() << std::endl; //
    std::cout << result[1].x() << "," << result[1].y() << std::endl; // 
    std::cout << result[2].x() << "," << result[2].y() << std::endl; 

    std::cout <<"Window 2 Test Started\n";
    
    Event event4;
    event4.position = Eigen::Vector2d(12.0,11.0);
    event4.timestamp = 1.7;

    Ti = 2.0;
    Event event5;
    event5.position = Eigen::Vector2d(12.2,11.1);
    event5.timestamp = 1.8;
    Event event6;
    event6.position = Eigen::Vector2d(12.4,11.2);
    event6.timestamp = 1.9;
    std::vector<Event> Window2;
    Window2.push_back(event4);
    Window2.push_back(event5);
    Window2.push_back(event6);

    feature.setEvents(Window2);
    feature.propagatePreviousEvents(Ti);
    const std::vector<Eigen::Vector2d>& result2 = feature.getLandmark();
    std::cout << result2[0].x() << "," << result2[0].y() << std::endl; //
    std::cout << result2[1].x() << "," << result2[1].y() << std::endl; // 
    std::cout << result2[2].x() << "," << result2[2].y() << std::endl; 

    std::cout <<"Window 3 Test Started\n";
    
    Event event7;
    event7.position = Eigen::Vector2d(14.0,12.0);
    event7.timestamp = 2.7;

    Ti = 3.0;
    Event event8;
    event8.position = Eigen::Vector2d(14.2,12.1);
    event8.timestamp = 2.8;
    Event event9;
    event9.position = Eigen::Vector2d(14.4,12.2);
    event9.timestamp = 2.9;
    std::vector<Event> Window3;
    Window3.push_back(event7);
    Window3.push_back(event8);
    Window3.push_back(event9);

    feature.setEvents(Window3);
    feature.propagatePreviousEvents(Ti);
    const std::vector<Eigen::Vector2d>& result3 = feature.getLandmark();
    std::cout << result3[0].x() << "," << result3[0].y() << std::endl; //
    std::cout << result3[1].x() << "," << result3[1].y() << std::endl; // 
    std::cout << result3[2].x() << "," << result3[2].y() << std::endl; 


}

