/*!
 * \author University of Minnesota
 * \author Aerospace Engineering and Mechanics
 * \copyright Copyright 2011 Regents of the University of Minnesota. All rights reserved.
 */

/// Standard function to initialize the navigation filter.
/*!
 * \sa get_nav(), close_nav()
 * \ingroup nav_fcns
*/
void init_researchNav(struct sensordata *sensorData_ptr,	///< pointer to sensorData structure
		struct mission *missionData_ptr,					///< pointer to the missionData structure
		struct nav *navData_ptr,			///< pointer to navData structure
		struct researchNav *researchNavData_ptr			///< pointer to navData structure
		);

/// Standard function to call the navigation filter.
/*!
 * \sa init_nav(), close_nav()
 * \ingroup nav_fcns
*/
void get_researchNav(struct sensordata *sensorData_ptr,	///< pointer to sensorData structure
		struct mission *missionData_ptr,					///< pointer to the missionData structure
		struct nav *navData_ptr,			///< pointer to navData structure
		struct researchNav *researchNavData_ptr			///< pointer to navData structure
		);

/// Standard function to close the navigation filter.
/*!
 * No input or return parameters
 * \sa get_nav(), init_nav()
 * \ingroup nav_fcns
*/
void close_researhcNav(void);

