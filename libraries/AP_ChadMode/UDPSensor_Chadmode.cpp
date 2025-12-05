#include <AP_ChadMode/UDPSensor_Chadmode.h>
#include <cstring> 
#include <AC_AttitudeControl/AC_PosControl.h>


CHAD_Sensor::CHAD_Sensor(AP_HAL::UARTDriver *port)
    : treated_so_far(0)   
{
    if (port != nullptr) {
        sensor_driver = port;
    } else {
        sensor_driver = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_CHAD, 0 );
    }
}


void CHAD_Sensor::init(){
    sensor_driver = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_CHAD, 0 );
};


ssize_t CHAD_Sensor::read(uint8_t *buf, uint16_t n) {
    if (sensor_driver == nullptr) {
        sensor_driver = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_CHAD, 0 );
        return -1;
    }

    return ((Linux::UARTDriver *)sensor_driver)->_device->read(buf, n);
}

void CHAD_Sensor::update_time(){

    last_timer_ms = AP_HAL::millis() ;

}

void CHAD_Sensor::update() {
    //static Logging_csv logout("log_decon.csv");
    if (sensor_driver == nullptr)
        return;

    //while (sensor_driver->available()) {
    ssize_t bytes_read = read(buffer + treated_so_far, size_pack - treated_so_far); 
    //logout.write("treated/read", treated_so_far, bytes_read);

    if (bytes_read < 0) {
    //    logout.write("caca");
        return;
    }

    treated_so_far += bytes_read;
    
    if (treated_so_far >= size_pack) {
       
      //logout.write("package", 
      //         get_kP_x(), 
      //         get_kI_x(),
      //         get_kD_x(),
      //         get_kP_y(), 
      //         get_kI_y(),
      //         get_kD_y(),
      //         get_kP_z(), 
      //         get_kI_z(),
      //         get_kD_z(),
      //         last_valid_package.delta_x,
      //         last_valid_package.delta_y,
      //         last_valid_package.delta_z,
      //         last_valid_package.pid_x_enabled,
      //         last_valid_package.pid_y_enabled,
      //         last_valid_package.pid_z_enabled,
      //         last_valid_package.nb_kp_ref,
      //         last_valid_package.nb_kp_cur,
      //         last_valid_package.nb_good,
      //         AP_HAL::millis());
//
        std::memcpy(&last_valid_package, buffer, size_pack);
        treated_so_far = 0;

        update_time();

        pid_x_enable = last_valid_package.pid_x_enabled;
        pid_y_enable = last_valid_package.pid_y_enabled;
        pid_z_enable = last_valid_package.pid_z_enabled;

        update_mean();

    }
    //}
}




float_t CHAD_Sensor::get_error(CHAD_Sensor::CHAD_Dir axe) const {

    switch (axe){
        case CHAD_Dir::X_AXIS :
            return last_mean_package.delta_x;

        case CHAD_Dir::Y_AXIS :
            return last_mean_package.delta_y;

        case CHAD_Dir::Z_AXIS :
            return last_mean_package.delta_z;
    }
    return 0.0f ;
}


void CHAD_Sensor::update_PID(AC_PID& pid, CHAD_Sensor::CHAD_Dir direction){
    float old_p = pid.get_p();
    float old_i = pid.get_i();
    float old_d = pid.get_d();

    switch (direction) {
        case CHAD_Dir::X_AXIS :
            if (pid_x_enable){
                pid.set_kP(get_kP_x());
                pid.set_kI(get_kI_x());
                pid.set_kD(get_kD_x());
            } else {
                pid.set_kP(0.0f);
                pid.set_kI(0.0f);
                pid.set_kD(0.0f);
            }

            if (old_p != get_kP_x() || old_i != get_kI_x() || old_d != get_kD_x()) {
                pid.reset_I();
            }
            break;

        case CHAD_Dir::Y_AXIS :
            if (pid_y_enable){
                pid.set_kP(get_kP_y());
                pid.set_kI(get_kI_y());
                pid.set_kD(get_kD_y());
            } else {
                pid.set_kP(0.0f);
                pid.set_kI(0.0f);
                pid.set_kD(0.0f);
                pid.reset_I();
            }
            if (old_p != get_kP_y() || old_i != get_kI_y() || old_d != get_kD_y()) {
                pid.reset_I();
            }
            break;
                
        case CHAD_Dir::Z_AXIS :
            if (pid_z_enable){
                pid.set_kP(get_kP_z());
                pid.set_kI(get_kI_z());
                pid.set_kD(get_kD_z());
            } else {
                pid.set_kP(0.0f);
                pid.set_kI(0.0f);
                pid.set_kD(0.0f);
                pid.reset_I();
            }
            if (old_p != get_kP_z() || old_i != get_kI_z() || old_d != get_kD_z()) {
                pid.reset_I();
            }
            break;
    }
}


float CHAD_Sensor::get_input_k(CHAD_Sensor::CHAD_Dir axe){
    switch (axe){
        case CHAD_Dir::X_AXIS :
            return last_valid_package.k_input_x ;

        case CHAD_Dir::Y_AXIS :
            return last_valid_package.k_input_y ;

        case CHAD_Dir::Z_AXIS :
            return last_valid_package.k_input_z ;
    }
    return 0.0f;
}


bool CHAD_Sensor::is_enable_PID(CHAD_Sensor::CHAD_Dir axe){
    switch (axe){
            case CHAD_Dir::X_AXIS :
                return pid_x_enable;

            case CHAD_Dir::Y_AXIS :
                return pid_y_enable ;

            case CHAD_Dir::Z_AXIS :
                return pid_z_enable ;
        }
        return false;
} 

void CHAD_Sensor::enable_PID(CHAD_Sensor::CHAD_Dir axe){
    switch (axe){
            case CHAD_Dir::X_AXIS :
                pid_x_enable=true;
                
                break;

            case CHAD_Dir::Y_AXIS :
                pid_y_enable=true;
                break;

            case CHAD_Dir::Z_AXIS :
                pid_z_enable=true ;
                break;
        }
} 

void CHAD_Sensor::disable_PID(CHAD_Sensor::CHAD_Dir axe){
    switch (axe){
            case CHAD_Dir::X_AXIS :
                pid_x_enable=false;
                break;

            case CHAD_Dir::Y_AXIS :
                pid_y_enable=false;
                break;

            case CHAD_Dir::Z_AXIS :
                pid_z_enable=false ;
                break;
        }
} 

void CHAD_Sensor::add( package& a, package& b){

            //on ne garde que les dernieres instructions sur les pid en supposant que a est postérieur à b   
            a.delta_x       += b.delta_x       ; 
            a.delta_y       += b.delta_y       ; 
            a.delta_z       += b.delta_z       ; 

            a.nb_kp_ref     += b.nb_kp_ref     ;  
            a.nb_kp_cur     += b.nb_kp_cur     ; 
            a.nb_good       += b.nb_good       ;
            
            a.kI_x       += b.kI_x             ;  
            a.kP_x       += b.kP_x             ; 
            a.kD_x       += b.kD_x             ;

            a.kI_y       += b.kI_y             ;  
            a.kP_y       += b.kP_y             ; 
            a.kD_y       += b.kD_y             ;

            a.kI_z       += b.kI_z             ;  
            a.kP_z       += b.kP_z             ; 
            a.kD_z       += b.kD_z             ;

        }

void CHAD_Sensor::divide( package& a, float f){

            a.delta_x       /= f; 
            a.delta_y       /= f; 
            a.delta_z       /= f;   

            a.nb_kp_ref     /= f;  
            a.nb_kp_cur     /= f; 
            a.nb_good       /= f;  

            a.kI_x       /= f;            ;  
            a.kP_x       /= f;            ; 
            a.kD_x       /= f;            ;
            a.kI_y       /= f;            ;  
            a.kP_y       /= f;            ; 
            a.kD_y       /= f;            ;
            a.kI_z       /= f;            ;  
            a.kP_z       /= f;            ; 
            a.kD_z       /= f;            ;
        };


void CHAD_Sensor::update_mean(){

    queue[pos_queue] = last_valid_package;
    
    if (nb_package_in_queue < nb_mean){
        nb_package_in_queue ++;
    }
    
    pos_queue = (pos_queue + 1) % nb_mean;

    package mean = {0};
    mean.pid_x_enabled = true;
    mean.pid_y_enabled = true;
    mean.pid_z_enabled = true;

    for (int i = 0; i < nb_package_in_queue; i++){
        add(mean,queue[i]);
    }
     
    if (nb_package_in_queue > 0){
        divide(mean,(float) nb_package_in_queue);
    }

    last_mean_package = mean ;
    
}
