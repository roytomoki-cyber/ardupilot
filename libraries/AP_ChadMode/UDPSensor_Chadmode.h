#pragma once

#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AC_PID/AC_PID.h>
#include <AP_Logcsv/logging_CSV.h>
#include <AP_HAL_Linux/UARTDriver.h>



class CHAD_Sensor{
    private :
        struct PACKED package {

            float kP_x;
            float kI_x;
            float kD_x;
            float k_input_x;

            float kP_y;
            float kI_y;
            float kD_y;
            float k_input_y;
            
            float kP_z;
            float kI_z;
            float kD_z;
            float k_input_z;

            float delta_x;
            float delta_y;
            float delta_z;

            bool pid_x_enabled;
            bool pid_y_enabled;
            bool pid_z_enabled;
            
            double nb_kp_ref;
            double nb_kp_cur;
            double nb_good;

        };

        static constexpr int size_pack = sizeof(package);

        uint8_t buffer[size_pack];
        size_t treated_so_far = 0;
       

        AP_HAL::UARTDriver *sensor_driver ;

        package last_valid_package;
        
        package last_mean_package;

        float last_timer_ms; 

        bool pid_x_enable ;

        bool pid_y_enable ;

        bool pid_z_enable ;



        static constexpr int nb_mean = 10 ;

        package queue[nb_mean] ; 

        int pos_queue = 0 ; 

        int nb_package_in_queue = 0;



	// TODO some boolean to indicate whether we have received any valid packages yet (or just a healthy flag)



    public :

        enum class CHAD_Dir {
                X_AXIS          = 0,
                Y_AXIS ,
                Z_AXIS 
            };
        
        CHAD_Sensor(AP_HAL::UARTDriver *port);
        
        void init();
        
        void update_time();

        ssize_t read(uint8_t *buf, uint16_t n);

        void update(); 

        void update_PID(AC_PID& pid, CHAD_Dir direction); 

        const package& get_data() const { return last_valid_package; };

        bool is_enable_PID(CHAD_Dir axe) ; 
        void enable_PID(CHAD_Dir axe);
        void disable_PID(CHAD_Dir axe);

        float get_kP_x() const { return last_valid_package.kP_x; };
        float get_kI_x() const { return last_valid_package.kI_x; };
        float get_kD_x() const { return last_valid_package.kD_x; };

        float get_kP_y() const { return last_valid_package.kP_y; };
        float get_kI_y() const { return last_valid_package.kI_y; };
        float get_kD_y() const { return last_valid_package.kD_y; };

        float get_kP_z() const { return last_valid_package.kP_z; };
        float get_kI_z() const { return last_valid_package.kI_z; };
        float get_kD_z() const { return last_valid_package.kD_z; };

        float get_input_k(CHAD_Dir axe);
        
        float get_last_time() { return last_timer_ms ;}

        
        
        //float_t get_error_x() {return last_valid_package.delta_x;};
        //float_t get_error_y() {return last_valid_package.delta_y;};  
        //float_t get_error_z() {return last_valid_package.delta_z;}; 


        float_t get_error(CHAD_Dir axe) const ;

        void update_mean() ;

        void add(package& a, package& b) ;
        void divide(package& a, float f) ;
    };
