#!/usr/bin/python

# report weather for specified flight

import argparse

from aurauas.flightdata import flight_loader, flight_interp
from aurauas.weather import forecast

parser = argparse.ArgumentParser(description='correlate movie data to flight data.')
parser.add_argument('--flight', help='load specified aura flight log')
parser.add_argument('--aura-flight', help='load specified aura flight log')
parser.add_argument('--px4-sdlog2', help='load specified px4 sdlog2 (csv) flight log')
parser.add_argument('--px4-ulog', help='load specified px4 ulog (csv) base path')
parser.add_argument('--umn-flight', help='load specified .mat flight log')
parser.add_argument('--sentera-flight', help='load specified sentera flight log')
parser.add_argument('--sentera2-flight', help='load specified sentera2 flight log')
args = parser.parse_args()

if args.flight:
    loader = 'aura'
    path = args.flight
elif args.aura_flight:
    loader = 'aura'
    path = args.aura_flight
elif args.px4_sdlog2:
    loader = 'px4_sdlog2'
    path = args.px4_sdlog2
elif args.px4_ulog:
    loader = 'px4_ulog'
    path = args.px4_ulog
elif args.sentera_flight:
    loader = 'sentera1'
    path = args.sentera_flight
elif args.sentera2_flight:
    loader = 'sentera2'
    path = args.sentera2_flight
elif args.umn_flight:
    loader = 'umn1'
    path = args.umn_flight
else:
    loader = None
    path = None
    
recal_file = None
data = flight_loader.load(loader, path, recal_file)
print "gps records:", len(data['gps'])
if len(data['gps']) == 0:
    print "not enough data loaded to continue."
    quit()

wx = forecast.Forecast()

print 'beginning of flight:'
unix_sec = data['gps'][0].unix_sec
lat = data['gps'][0].lat
lon = data['gps'][0].lon
wx.query(lat, lon, unix_sec)
wx.report()
