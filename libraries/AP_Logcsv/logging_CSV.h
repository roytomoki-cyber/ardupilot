#pragma once
#include <fstream>
#include <string>

class Logging_csv{
    public:

        Logging_csv(std::string file){
            log_out = std::ofstream(file);
        }

        //ecrit dans un ficher file (csv) les data a,b,c,d,e 
        void write(float a, float b, float c, float d, float e);

        std::ofstream log_out;
};