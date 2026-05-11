#pragma once

#include <opencv2/opencv.hpp>


#include "types.h"

class DisplayBoard {
    private:
        int board_width; // width of display board(pixels) 
        int board_height; // height of display board(pixels)
        float scale_factor; // how many pixels to display per meter 
        cv::Mat debug_field; // display board
        FieldDimensions fd; // size of football field
        std::vector<FieldMarker> markers; // markers to display
        bool window_initialized = false;

        cv::Point rotateVector(int x, int y, float theta);
        void ensureWindow();

    public:

        void init(FieldDimensions _fd, int board_width = 760, int board_height = 560, float scale_factor = 75);
    
        void drawFieldBoard();

        void display();

        void clearMarkers();

        void addMarker(string type, double x, double y);
    
        void displayMarkers();
    
        void displayRobotPose(double x, double y, double theta);

        void pumpEvents(int delay_ms = 1);
    
};
    
