import data_aura
import data_sentera
import data_sentera2
import data_umn

def load(args):
    # aura_flight=None, sentera_flight=None, umn_flight=None)
    if args.flight:
        imu_data, gps_data, filter_data = data_aura.load(args.flight, args.recalibrate)
    elif args.aura_flight:
        imu_data, gps_data, filter_data = data_aura.load(args.aura_flight, args.recalibrate)
    elif args.sentera_flight:
        imu_data, gps_data, filter_data = data_sentera.load(args.sentera_flight)
    elif args.sentera2_flight:
        imu_data, gps_data, filter_data = data_sentera2.load(args.sentera2_flight)
    elif args.umn_flight:
        imu_data, gps_data, filter_data = data_umn.load(args.umn_flight)
    else:
        print "no valid input file / dir specified"
        imu_data, gps_data, filter_data = [], [], []
    return imu_data, gps_data, filter_data
