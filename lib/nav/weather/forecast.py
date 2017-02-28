# forecast.io interface

import datetime
import os
import urllib, json

mph2kt = 0.868976
mb2inhg = 0.0295299830714

class Forecast:
    def __init__(self):
        self.apikey = None
        self.data = None
        try:
            home = os.path.expanduser("~")
            dotfile = os.path.join(home, '.forecastio')
            f = open(dotfile, 'rb')
            self.apikey = f.read().rstrip()
        except:
            print "you must sign up for a free apikey at forecast.io and insert it as a single line inside a file called ~/.forecastio (with no other text in the file)"

    def query(self, lat, lon, unix_sec):
        if not self.apikey:
            print "Cannot lookup weather because no forecastio apikey found."
            return None
        elif unix_sec < 1:
            print "Cannot lookup weather without valid gps time."
            return None
        else:
            d = datetime.datetime.utcfromtimestamp(unix_sec)
            print d.strftime("%Y-%m-%d-%H:%M:%S")
            url = 'https://api.darksky.net/forecast/' + self.apikey + '/%.8f,%.8f,%.d' % (lat, lon, unix_sec)
            response = urllib.urlopen(url)
            self.data = json.loads(response.read())
            return self.data

    def report(self, data=None):
        if not data:
            data = self.data
        if 'currently' in data:
            currently = data['currently']
            if 'icon' in currently:
                icon = currently['icon']
                print 'Summary:', icon
            if 'temperature' in currently:
                tempF = currently['temperature']
                tempC = (tempF - 32.0) * 5 / 9
                print 'Temp:', '%.1f F' % tempF, '(%.1f C)' % tempC
            if 'dewPoint' in currently:
                tempF = currently['dewPoint']
                tempC = (tempF - 32.0) * 5 / 9
                print 'Dewpoint:', '%.1f F' % tempF, '(%.1f C)' % tempC
            if 'humidity' in currently:
                hum = currently['humidity']
                print 'Humidity:', '%.0f%%' % (hum * 100.0)
            if 'pressure' in currently:
                mbar = currently['pressure']
                inhg = mbar * mb2inhg
                print 'Pressure:', '%.2f inhg' % inhg, '(%.1f mbar)' % mbar
            if 'windSpeed' in currently:
                wind_mph = currently['windSpeed']
                wind_kts = wind_mph * mph2kt
            else:
                wind_mph = 0
                wind_kts = 0
            if 'windBearing' in currently:
                wind_deg = currently['windBearing']
            else:
                wind_deg = 0
            print "Wind %d deg @ %.1f kt (%.1f mph) @ " % (wind_deg, wind_kts, wind_mph, )
            if 'visibility' in currently:
                vis = currently['visibility']
                print 'Visibility:', '%.1f miles' % vis
            if 'cloudCover' in currently:
                cov = currently['cloudCover']
                print 'Cloud Cover:', '%.0f%%' % (cov * 100.0)
