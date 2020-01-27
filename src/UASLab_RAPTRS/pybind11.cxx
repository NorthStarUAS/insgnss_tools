#ifdef HAVE_PYBIND11

#include <pybind11/pybind11.h>
namespace py = pybind11;

#include <math.h>

#include "../nav_common/structs.hxx"

#include "uNavINS.h"

const double D2R = M_PI / 180.0; // degrees to radians

// this is a glue class to bridge between the existing python API and
// the actual uNavINS class API.  This could be handled other ways,
// but it also hides the eigen3 usage in the uNavINS interfaces

class APIHelper {

public:

    APIHelper() {
	filt.Configure();
    }
    ~APIHelper() {}

    // set/get error characteristics of navigation sensors
    void set_config(NAVconfig config) {
        config_save = config;
        
        // set config values
        filt.Set_AccelNoise( config.sig_w_ax );
        filt.Set_GyroNoise( config.sig_w_gx );
        filt.Set_AccelMarkov( config.sig_a_d );
        filt.Set_AccelTau( config.tau_a );
        filt.Set_GyroMarkov( config.sig_g_d );
        filt.Set_GyroTau( config.tau_g );
        filt.Set_GpsPosNoiseNE( config.sig_gps_p_ne );
        filt.Set_GpsPosNoiseD( config.sig_gps_p_d );
        filt.Set_GpsVelNoiseNE( config.sig_gps_v_ne );
        filt.Set_GpsVelNoiseD( config.sig_gps_v_d );
        
        // commit these values
        filt.Configure();
    }
    
    NAVconfig get_config() {
        return config_save;
    }
    
    void default_config() {
        // no-op
    }

    // main interface
    void init(IMUdata imu, GPSdata gps) {
        Vector3f wMeas_rps( imu.p, imu.q, imu.r );
        Vector3f aMeas_mps2( imu.ax, imu.ay, imu.az );
        Vector3f magMeas( imu.hx, imu.hy, imu.hz );
        Vector3d pMeas_D_rrm( gps.lat*D2R, gps.lon*D2R, gps.alt );
        Vector3f vMeas_L_mps( gps.vn, gps.ve, gps.vd );
        imu_save = imu;
        gps_save = gps;
        printf("uNavINS Init: %.8f, %.8f %.2f\n", gps.lat, gps.lon, gps.alt);
        filt.Initialize(wMeas_rps, aMeas_mps2, magMeas, pMeas_D_rrm,
                        vMeas_L_mps);
    }
    
    void time_update(IMUdata imu) {
        Vector3f wMeas_rps( imu.p, imu.q, imu.r );
        Vector3f aMeas_mps2( imu.ax, imu.ay, imu.az );
        Vector3f magMeas( imu.hx, imu.hy, imu.hz );
        Vector3d pMeas_D_rrm( gps_save.lat*D2R, gps_save.lon*D2R, gps_save.alt );
        Vector3f vMeas_L_mps( gps_save.vn, gps_save.ve, gps_save.vd );
        imu_save = imu;
        filt.Update((uint64_t)(imu.time * 1e+6),
                    (unsigned long)gps_save.time*100,
                    wMeas_rps, aMeas_mps2, magMeas, pMeas_D_rrm, vMeas_L_mps);
    }
    void measurement_update(IMUdata imu, GPSdata gps) {
        Vector3f wMeas_rps( imu.p, imu.q, imu.r );
        Vector3f aMeas_mps2( imu.ax, imu.ay, imu.az );
        Vector3f magMeas( imu.hx, imu.hy, imu.hz );
        Vector3d pMeas_D_rrm( gps.lat*D2R, gps.lon*D2R, gps.alt );
        Vector3f vMeas_L_mps( gps.vn, gps.ve, gps.vd );
        imu_save = imu;
        gps_save = gps;
        filt.Update((uint64_t)(imu.time * 1e+6),
                    (unsigned long)gps.time*100,
                    wMeas_rps, aMeas_mps2, magMeas, pMeas_D_rrm, vMeas_L_mps);
    }
    
    NAVdata get_nav() {
        NAVdata result;
        result.time = imu_save.time;
        Vector3d PosEst_rrm = filt.Get_PosEst();
        result.lat = PosEst_rrm[0];
        result.lon = PosEst_rrm[1];
        result.alt = PosEst_rrm[2];
        Vector3f VelEst_mps = filt.Get_VelEst();
        result.vn = VelEst_mps[0];
        result.ve = VelEst_mps[1];
        result.vd = VelEst_mps[2];
        Vector3f OrientEst_rad = filt.Get_OrientEst();
        result.phi = OrientEst_rad[0];
        result.the = OrientEst_rad[1];
        result.psi = OrientEst_rad[2];
        Vector3f AccelBias_mps2 = filt.Get_AccelBias();
        result.abx = AccelBias_mps2[0];
        result.aby = AccelBias_mps2[1];
        result.abz = AccelBias_mps2[2];
        Vector3f GyroBias_rps = filt.Get_RotRateBias();
        result.gbx = GyroBias_rps[0];
        result.gby = GyroBias_rps[1];
        result.gbz = GyroBias_rps[2];

        return result;
    }

private:

    NAVconfig config_save;
    IMUdata imu_save;
    GPSdata gps_save;
    uNavINS filt;
    
};

PYBIND11_MODULE(uNavINS, m) {
    py::class_<APIHelper>(m, "uNavINS")
        .def(py::init<>())
        .def("set_config", &APIHelper::set_config)
        .def("init", &APIHelper::init)
        .def("time_update", &APIHelper::time_update)
        .def("measurement_update", &APIHelper::measurement_update)
        .def("get_nav", &APIHelper::get_nav)
    ;
}

#endif
