import aura
import px4_sdlog2
import px4_ulog
import sentera
import sentera2
import umn_mat

def load(loader, path, recal_file=None):
    flight_data = {}
    
    if loader == 'aura':
        flight_data = aura.load(path, recal_file)
    elif loader == 'px4_sdlog2':
        flight_data = px4_sdlog2.load(path)
    elif loader == 'px4_ulog':
        flight_data = px4_ulog.load(path)
    elif loader == 'sentera1':
        imu_data, gps_data, filter_data = sentera.load(path)
    elif loader == 'sentera2':
        imu_data, gps_data, air_data, filter_data = sentera2.load(path)
    elif loader == 'umn1':
        flight_data = umn_mat.load(path)
    else:
        print 'loader:', loader, 'path:', path
        print "no valid input file / dir specified"
        
    return flight_data
