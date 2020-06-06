// Compute local gravity as a function of latitude (radians) and
// altitude (m).  Returns the local gravity vector in ned coordinates.
// In other words, it computes the magnitude of the of local gravity
// at the wgs84 reference elliposid using the Somigliana model and
// makes corrections for altitude. 

Vector3f local_gravity( double lat_rad, float alt_m );
