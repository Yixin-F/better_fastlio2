// Author:   Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

#pragma once

#include <ctime>
#include <iostream>
#include <string>
#include <cstdlib>
#include <chrono>

class TicToc
{
public:
    TicToc()
    {
        tic();
    }

    TicToc( std::string _about_task )
    {
        disp_ = _about_task;
        tic();
    }

    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    double toc( std::string _about_task)
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        double elapsed_ms = elapsed_seconds.count() * 1000;

        // if( disp_ )
        // {
        //   std::cout.precision(3); // 10 for sec, 3 for ms 
        //   std::cout << _about_task << ": " << elapsed_ms << " msec." << std::endl;
        // }
        return elapsed_ms;
    }

private:  
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::string disp_;
};
