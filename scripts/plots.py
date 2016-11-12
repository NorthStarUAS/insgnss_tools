from matplotlib import pyplot as plt
import numpy as np

class Plots():
    def __init__(self, plotname):
        self.plotname = plotname
        
        self.att_fig, self.att_ax = plt.subplots(3,1, sharex=True)
        self.att_ax[0].grid()
        self.att_ax[1].grid()
        self.att_ax[2].grid()
        self.att_ax[2].set_xlabel('Time (sec)', weight='bold')

        self.vel_fig, self.vel_ax = plt.subplots(3,1, sharex=True)
        self.vel_ax[0].grid()
        self.vel_ax[1].grid()
        self.vel_ax[2].grid()
        self.vel_ax[2].set_xlabel('Time (sec)', weight='bold')

        self.pos_fig, self.pos_ax = plt.subplots(3,1, sharex=True)
        self.pos_ax[0].grid()
        self.pos_ax[1].grid()
        self.pos_ax[2].grid()
        self.pos_ax[2].set_xlabel('Time (sec)', weight='bold')

        self.bias_fig, self.bias_ax = plt.subplots(3,2, sharex=True)

        # turn on interactive mode
        plt.ion()

    def update(self, data_dict,
               label='Unknown', ls='-', marker=' ', c='g', alpha=0.5):
        r2d = np.rad2deg
        nsig = 3

        t_flight = data_dict.time

        # Roll Plot
        if len(data_dict.phi):
            phi = data_dict.phi
            self.att_ax[0].set_ylabel('Roll (deg)', weight='bold')
            self.att_ax[0].plot(t_flight, r2d(phi), label=label, ls=ls, c=c, alpha=alpha)

        # Pitch Plot
        if len(data_dict.the):
            the = data_dict.the
            self.att_ax[1].set_ylabel('Pitch (deg)', weight='bold')
            self.att_ax[1].plot(t_flight, r2d(the), label=label, ls=ls, c=c, alpha=alpha)

        # Yaw Plot
        if len(data_dict.psi):
            psi = data_dict.psi
            self.att_ax[2].set_title(self.plotname, fontsize=10)
            self.att_ax[2].set_ylabel('Yaw (deg)', weight='bold')
            self.att_ax[2].plot(t_flight, r2d(psi), label=label, ls=ls, c=c, alpha=alpha)

        self.att_ax[2].legend(loc=1)
        self.att_fig.canvas.draw()

        # vn Plot
        vn = data_dict.vn
        self.vel_ax[0].set_title(self.plotname, fontsize=10)
        self.vel_ax[0].set_ylabel('vn (mps)', weight='bold')
        self.vel_ax[0].plot(t_flight, vn, ls=ls, marker=marker, label=label, c=c, lw=2, alpha=alpha)
        self.vel_ax[0].legend(loc=0)

        # ve Plot
        ve = data_dict.ve
        self.vel_ax[1].set_ylabel('ve (mps)', weight='bold')
        self.vel_ax[1].plot(t_flight, ve, ls=ls, marker=marker, label=label, c=c, lw=2, alpha=alpha)

        # vd Plot
        vd = data_dict.vd
        self.vel_ax[2].set_ylabel('vd (mps)', weight='bold')
        self.vel_ax[2].plot(t_flight, vd, ls=ls, marker=marker, label=label, c=c, lw=2, alpha=alpha)

        self.vel_fig.canvas.draw()

        # lat plot
        lat = data_dict.lat
        self.pos_ax[0].set_title('Position')
        self.pos_ax[0].set_ylabel('Lat (deg)', weight='bold')
        self.pos_ax[0].plot(t_flight, r2d(lat), ls=ls, marker=marker, label=label, c=c, lw=2, alpha=alpha)
        self.pos_ax[0].legend(loc=0)

        # lon plot
        lon = data_dict.lon
        self.pos_ax[1].set_ylabel('Lon (deg)', weight='bold')
        self.pos_ax[1].plot(t_flight, r2d(lon), ls=ls, marker=marker, label=label, c=c, lw=2, alpha=alpha)

        # alt plot
        alt = data_dict.alt
        self.pos_ax[2].set_ylabel('Alt (m)', weight='bold')
        self.pos_ax[2].plot(t_flight, alt, ls=ls, marker=marker, label=label, c=c, lw=2, alpha=alpha)

        self.pos_fig.canvas.draw()

        # Top View (Longitude vs. Latitude) Plot
        # if FLAG_PLOT_GROUNDTRACK:
        #     navlat = data_dict1.lat
        #     navlon = data_dict1.lon
        #     nav_maglat = data_dict2.lat
        #     nav_maglon = data_dict2.lon
        #     plt.figure()
        #     plt.title(self.plotname, fontsize=10)
        #     plt.ylabel('LATITUDE (DEGREES)', weight='bold')
        #     plt.xlabel('LONGITUDE (DEGREES)', weight='bold')
        #     plt.plot(lon_gps, lat_gps, '*', label='GPS Sensor', c='g', lw=2, alpha=.5)
        #     plt.plot(r2d(navlon_flight), r2d(navlat_flight), label='On Board', c='k', lw=2, alpha=.5)
        #     plt.plot(r2d(navlon), r2d(navlat), label=filter1.name, c='r', lw=2, alpha=.8)
        #     plt.plot(r2d(nav_maglon), r2d(nav_maglat), label=filter2.name, c='b', lw=2, alpha=.8)
        #     plt.grid()
        #     plt.legend(loc=0)

        if len(data_dict.p_bias) and len(data_dict.q_bias) and len(data_dict.r_bias):
            # Gyro Biases
            self.bias_ax[0,0].set_ylabel('p Bias (deg/s)', weight='bold')
            self.bias_ax[0,0].plot(t_flight, r2d(data_dict.p_bias), label=label, c=c)
            self.bias_ax[0,0].set_xlabel('Time (secs)', weight='bold')
            self.bias_ax[0,0].grid()

            self.bias_ax[1,0].set_ylabel('q Bias (deg/s)', weight='bold')
            self.bias_ax[1,0].plot(t_flight, r2d(data_dict.q_bias), label=label, c=c)
            self.bias_ax[1,0].set_xlabel('Time (secs)', weight='bold')
            self.bias_ax[1,0].grid()

            self.bias_ax[2,0].set_ylabel('r Bias (deg/s)', weight='bold')
            self.bias_ax[2,0].plot(t_flight, r2d(data_dict.r_bias), label=label, c=c)
            self.bias_ax[2,0].set_xlabel('Time (secs)', weight='bold')
            self.bias_ax[2,0].grid()

        if len(data_dict.ax_bias) and len(data_dict.ay_bias) and len(data_dict.az_bias):
            # Accel Biases
            self.bias_ax[0,1].set_ylabel('ax Bias (m/s^2)', weight='bold')
            self.bias_ax[0,1].plot(t_flight, data_dict.ax_bias, label=label, c=c)
            self.bias_ax[0,1].set_xlabel('Time (secs)', weight='bold')
            self.bias_ax[0,1].grid()

            self.bias_ax[1,1].set_ylabel('ay Bias (m/s^2)', weight='bold')
            self.bias_ax[1,1].plot(t_flight, data_dict.ay_bias, label=label, c=c)
            self.bias_ax[1,1].set_xlabel('Time (secs)', weight='bold')
            self.bias_ax[1,1].grid()

            self.bias_ax[2,1].set_ylabel('az Bias (m/s^2)', weight='bold')
            self.bias_ax[2,1].plot(t_flight, data_dict.az_bias, label=label, c=c)
            self.bias_ax[2,1].set_xlabel('Time (secs)', weight='bold')
            self.bias_ax[2,1].grid()
            self.bias_ax[2,1].legend(loc=1)

        self.bias_fig.canvas.draw()

        # update plots and allow a short amount of interaction
        plt.pause(0.25)

    def explore(self):
        # turn off interactive plots and pause here to fully explore
        # the plot windows. This function will return when all the
        # plot windows are closed (dismissed.)
        plt.ioff()    
        plt.show()


