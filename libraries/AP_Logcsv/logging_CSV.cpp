#include <fstream>
#include <string>
#include "logging_CSV.h"

void Logging_csv::write(float a,float b,float c,float d,float e){

    if(Logging_csv::log_out.is_open()){
        Logging_csv::log_out<<a<<";";
        Logging_csv::log_out<<b<<";";
        Logging_csv::log_out<<c<<";";
        Logging_csv::log_out<<d<<";";
        Logging_csv::log_out<<e<<std::endl;
    }
}