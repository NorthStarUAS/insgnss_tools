#include <pybind11/pybind11.h>
namespace py = pybind11;

#include <math.h>

#include "../nav/nav_structs.h"

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
        // set config values
        filt.Set_AccelSigma( config.sig_w_ax );
        filt.Set_AccelMarkov( config.sig_a_d );
        filt.Set_AccelTau( config.tau_a );
        filt.Set_RotRateSigma( config.sig_w_gx );
        filt.Set_RotRateMarkov( config.sig_g_d );
        filt.Set_RotRateTau( config.tau_g );
        filt.Set_PosSigmaNE( config.sig_gps_p_ne );
        filt.Set_PosSigmaD( config.sig_gps_p_d );
        filt.Set_VelSigmaNE( config.sig_gps_v_ne );
        filt.Set_VelSigmaD( config.sig_gps_v_d );

        // commit these values
        filt.Configure();
    }

    void update(IMUdata imu, GPSdata gps) {
        Vector3f wMeas_rps( imu.p, imu.q, imu.r );
        Vector3f aMeas_mps2( imu.ax, imu.ay, imu.az );
        Vector3f magMeas( imu.hx, imu.hy, imu.hz );
        Vector3d pMeas_D_rrm( gps.lat*D2R, gps.lon*D2R, gps.alt );
        Vector3f vMeas_L_mps( gps.vn, gps.ve, gps.vd );
        current_time = imu.time_sec;
        if ( ! filt.Initialized() ) {
            filt.Initialize(wMeas_rps, aMeas_mps2, magMeas, pMeas_D_rrm, vMeas_L_mps);
        }
        filt.Update((uint64_t)(imu.time_sec * 1e+6),
                    (unsigned long)(gps.time_sec * 100),
                    wMeas_rps, aMeas_mps2, magMeas, pMeas_D_rrm, vMeas_L_mps);
    }

    NAVdata get_nav() {
        NAVdata result;
        result.time_sec = current_time;
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

PYBIND11_MODULE(uNavINS, m) {
    py::class_<APIHelper>(m, "uNavINS", py::module_local())
        .def(py::init<>())
        .def("set_config", &APIHelper::set_config)
        .def("update", &APIHelper::update)
        .def("get_nav", &APIHelper::get_nav)
    ;
}
