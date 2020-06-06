#include <pybind11/pybind11.h>
namespace py = pybind11;

#include <math.h>

#include "../nav_common/structs.h"

#include "uNavINS.h"

const double D2R = M_PI / 180.0; // degrees to radians

// this is a glue class to bridge between the existing python API and
// the actual uNavINS class API.  This could be handled other ways,
// but it also hides the eigen3 usage in the uNavINS interfaces

class APIHelper {

public:

    APIHelper() {}
    ~APIHelper() {}

    // set/get error characteristics of navigation sensors
    void set_config(NAVconfig config) {
        // set config values
        filt.setSig_W_A( config.sig_w_ax );
        filt.setSig_W_G( config.sig_w_gx );
        filt.setSig_A_D( config.sig_a_d );
        filt.setTau_A( config.tau_a );
        filt.setSig_G_D( config.sig_g_d );
        filt.setTau_G( config.tau_g );
        filt.setSig_GPS_P_NE( config.sig_gps_p_ne );
        filt.setSig_GPS_P_D( config.sig_gps_p_d );
        filt.setSig_GPS_V_NE( config.sig_gps_v_ne );
        filt.setSig_GPS_V_D( config.sig_gps_v_d );
    }
    
    void update(IMUdata imu, GPSdata gps) {
        current_time = imu.time;
        filt.update((uint64_t)(imu.time * 1e+6),
                    (unsigned long)(gps.time * 100),
                    gps.vn, gps.ve, gps.vd,
                    gps.lat*D2R, gps.lon*D2R, gps.alt,
                    imu.p, imu.q, imu.r,
                    imu.ax, imu.ay, imu.az,
                    imu.hx, imu.hy, imu.hz);
    }
    
    NAVdata get_nav() {
        NAVdata result;
        result.time = current_time;
        result.lat = filt.getLatitude_rad();
        result.lon = filt.getLongitude_rad();
        result.alt = filt.getAltitude_m();
        result.vn = filt.getVelNorth_ms();
        result.ve = filt.getVelEast_ms();
        result.vd = filt.getVelDown_ms();
        result.phi = filt.getRoll_rad();
        result.the = filt.getPitch_rad();
        result.psi = filt.getHeading_rad();
        result.abx = filt.getAccelBiasX_mss();
        result.aby = filt.getAccelBiasY_mss();
        result.abz = filt.getAccelBiasZ_mss();
        result.gbx = filt.getGyroBiasX_rads();
        result.gby = filt.getGyroBiasX_rads();
        result.gbz = filt.getGyroBiasX_rads();

        // these values currently aren't supported outputs from this
        // version of the ekf
        result.Pp0 = result.Pp1 = result.Pp2 = 0.0;
        result.Pv0 = result.Pv1 = result.Pv2 = 0.0;
        result.Pa0 = result.Pa1 = result.Pa2 = 0.0;
        result.Pabx = result.Paby = result.Pabz = 0.0;
        result.Pgbx = result.Pgby = result.Pgbz = 0.0;

        return result;
    }

private:

    float current_time;
    uNavINS filt;
    
};

PYBIND11_MODULE(uNavINS_BFS, m) {
    py::class_<APIHelper>(m, "uNavINS_BFS", py::module_local())
        .def(py::init<>())
        .def("set_config", &APIHelper::set_config)
        .def("update", &APIHelper::update)
        .def("get_nav", &APIHelper::get_nav)
    ;
}
