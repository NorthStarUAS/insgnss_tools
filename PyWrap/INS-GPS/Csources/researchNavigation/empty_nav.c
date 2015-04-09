/*!
 * \author University of Minnesota
 * \author Aerospace Engineering and Mechanics
 * \copyright Copyright 2011 Regents of the University of Minnesota. All rights reserved.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "../globaldefs.h"
#include "../utils/matrix.h"

#include "../navigation/nav_functions.h"
#include "researchnav_interface.h"

void init_researchNav(struct sensordata *sensorData_ptr, struct mission *missionData_ptr, struct nav *navData_ptr, struct researchNav *researchNavData_ptr){

	researchNavData_ptr->lat = sensorData_ptr->gpsData_ptr->lat*D2R;
	researchNavData_ptr->lon = sensorData_ptr->gpsData_ptr->lon*D2R;
	researchNavData_ptr->alt = sensorData_ptr->gpsData_ptr->alt;
	
	researchNavData_ptr->vn = sensorData_ptr->gpsData_ptr->vn;
	researchNavData_ptr->ve = sensorData_ptr->gpsData_ptr->ve;
	researchNavData_ptr->vd = sensorData_ptr->gpsData_ptr->vd;

	researchNavData_ptr->the = 8*D2R;
	researchNavData_ptr->phi = 0*D2R;
	researchNavData_ptr->psi = 90.0*D2R;

	researchNavData_ptr->ab[0] = 0.0;
	researchNavData_ptr->ab[1] = 0.0; 
	researchNavData_ptr->ab[2] = 0.0;

	researchNavData_ptr->gb[0] = sensorData_ptr->imuData_ptr->p;
	researchNavData_ptr->gb[1] = sensorData_ptr->imuData_ptr->q;
	researchNavData_ptr->gb[2] = sensorData_ptr->imuData_ptr->r;
	
	researchNavData_ptr->err_type = data_valid;
	//send_status("Research NAV filter initialized");
}

// Main get_nav filter function
void get_researchNav(struct sensordata *sensorData_ptr, struct mission *missionData_ptr, struct nav *navData_ptr, struct researchNav *researchNavData_ptr){
	
	researchNavData_ptr->lat = sensorData_ptr->gpsData_ptr->lat*D2R;
	researchNavData_ptr->lon = sensorData_ptr->gpsData_ptr->lon*D2R;
	researchNavData_ptr->alt = sensorData_ptr->gpsData_ptr->alt;
	
	researchNavData_ptr->vn = sensorData_ptr->gpsData_ptr->vn;
	researchNavData_ptr->ve = sensorData_ptr->gpsData_ptr->ve;
	researchNavData_ptr->vd = sensorData_ptr->gpsData_ptr->vd;

	researchNavData_ptr->the = 8*D2R;
	researchNavData_ptr->phi = 0*D2R;
	researchNavData_ptr->psi = 90.0*D2R;

	researchNavData_ptr->ab[0] = 0.0;
	researchNavData_ptr->ab[1] = 0.0; 
	researchNavData_ptr->ab[2] = 0.0;
	
	researchNavData_ptr->gb[0] = sensorData_ptr->imuData_ptr->p;
	researchNavData_ptr->gb[1] = sensorData_ptr->imuData_ptr->q;
	researchNavData_ptr->gb[2] = sensorData_ptr->imuData_ptr->r;
	
	researchNavData_ptr->err_type = gps_aided;
	//send_status("Research NAV filter initialized");
}

void close_researchNav(void){
	
}
