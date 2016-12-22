import aura
import sentera
import sentera2
import umn_mat

def load(args):
    flight_data = {}
    
    if 'recalibrate' in args:
        recal_file = args.recalibrate
    else:
        recal_file = None
        
    if args.flight:
        flight_data = aura.load(args.flight, recal_file)
    elif args.aura_flight:
        flight_data = aura.load(args.aura_flight, recal_file)
    elif args.sentera_flight:
        imu_data, gps_data, filter_data = sentera.load(args.sentera_flight)
    elif args.sentera2_flight:
        imu_data, gps_data, air_data, filter_data = sentera2.load(args.sentera2_flight)
    elif args.umn_flight:
        imu_data, gps_data, air_data, filter_data = umn_mat.load(args.umn_flight)
    else:
        print "no valid input file / dir specified"
        
    return flight_data
