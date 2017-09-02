import copy
import numpy as np
import os.path
import sys

from props import PropertyNode, root, getNode
import props_json
import props_xml

import nav.structs

class Calibration():
    def __init__(self):
        self.valid = False
        # defined temp range
        self.min_temp = 27.0
        self.max_temp = 27.0
        # temp vs. bias fit function coefficients
        self.p_bias = np.array([0.0, 0.0, 0.0])
        self.q_bias = np.array([0.0, 0.0, 0.0])
        self.r_bias = np.array([0.0, 0.0, 0.0])
        self.ax_bias = np.array([0.0, 0.0, 0.0])
        self.ay_bias = np.array([0.0, 0.0, 0.0])
        self.az_bias = np.array([0.0, 0.0, 0.0])
        # scaling factor
        self.p_scale = np.array([0.0, 0.0, 1.0])
        self.q_scale = np.array([0.0, 0.0, 1.0])
        self.r_scale = np.array([0.0, 0.0, 1.0])
        self.ax_scale = np.array([0.0, 0.0, 1.0])
        self.ay_scale = np.array([0.0, 0.0, 1.0])
        self.az_scale = np.array([0.0, 0.0, 1.0])
        self.mag_affine = np.identity(4)
        self.mag_affine_inv = np.linalg.inv(self.mag_affine)

    # load/parse an xml calibration file
    def load(self, cal_file):
        config = PropertyNode()
        try:
            name, ext = os.path.splitext(cal_file)
            if ext == '.json':
                props_json.load(cal_file, config)
                self.valid = True
            elif ext == '.xml':
                props_xml.load(cal_file, config)
                self.valid = True
            else:
                return False
        except:
            print cal_file + ": load error:\n" + str(sys.exc_info()[1])
            return False

        root.pretty_print()
        
        self.min_temp = config.getFloat('min_temp_C')
        self.max_temp = config.getFloat('max_temp_C')
        
        node = config.getChild('p')
        if node:
            p1, p2, p3 = node.getString('bias').split()
            self.p_bias = np.array([p1, p2, p3], dtype=np.float64)
            p1, p2, p3 = node.getString('scale').split()
            self.p_scale = np.array([p1, p2, p3], dtype=np.float64)

        node = config.getChild('q')
        if node:
            p1, p2, p3 = node.getString('bias').split()
            self.q_bias = np.array([p1, p2, p3], dtype=np.float64)
            p1, p2, p3 = node.getString('scale').split()
            self.q_scale = np.array([p1, p2, p3], dtype=np.float64)

        node = config.getChild('r')
        if node:
            p1, p2, p3 = node.getString('bias').split()
            self.r_bias = np.array([p1, p2, p3], dtype=np.float64)
            p1, p2, p3 = node.getString('scale').split()
            self.r_scale = np.array([p1, p2, p3], dtype=np.float64)

        node = config.getChild('ax')
        if node:
            p1, p2, p3 = node.getString('bias').split()
            self.ax_bias = np.array([p1, p2, p3], dtype=np.float64)
            p1, p2, p3 = node.getString('scale').split()
            self.ax_scale = np.array([p1, p2, p3], dtype=np.float64)

        node = config.getChild('ay')
        if node:
            p1, p2, p3 = node.getString('bias').split()
            self.ay_bias = np.array([p1, p2, p3], dtype=np.float64)
            p1, p2, p3 = node.getString('scale').split()
            self.ay_scale = np.array([p1, p2, p3], dtype=np.float64)

        node = config.getChild('az')
        if node:
            p1, p2, p3 = node.getString('bias').split()
            self.az_bias = np.array([p1, p2, p3], dtype=np.float64)
            p1, p2, p3 = node.getString('scale').split()
            self.az_scale = np.array([p1, p2, p3], dtype=np.float64)

        tokens = config.getString('mag_affine').split()
        if len(tokens) == 16:
            r = 0
            c = 0
            for i, x in enumerate(tokens):
                self.mag_affine[r,c] = float(x)
                c += 1
                if c > 3:
                    c = 0
                    r += 1
            self.mag_affine_inv = np.linalg.inv(self.mag_affine)
        else:
            print "mag_affine requires 16 values"
        #print 'mag_affine:\n', self.mag_affine
        #print 'mag_affine_inv:\n', self.mag_affine_inv

        return True
    
    # save a configuration file
    def save(self, cal_file):
        config = PropertyNode()
        config.setFloat('min_temp_C', self.min_temp)
        config.setFloat('max_temp_C', self.max_temp)

        node = config.getChild('p', create=True)
        p = self.p_bias
        node.setString('bias', "%.8f %.8f %.8f" % (p[0], p[1], p[2]))
        p = self.p_scale
        node.setString('scale', "%.8f %.8f %.8f" % (p[0], p[1], p[2]))

        node = config.getChild('q', create=True)
        p = self.q_bias
        node.setString('bias', "%.8f %.8f %.8f" % (p[0], p[1], p[2]))
        p = self.q_scale
        node.setString('scale', "%.8f %.8f %.8f" % (p[0], p[1], p[2]))

        node = config.getChild('r', create=True)
        p = self.r_bias
        node.setString('bias', "%.8f %.8f %.8f" % (p[0], p[1], p[2]))
        p = self.r_scale
        node.setString('scale', "%.8f %.8f %.8f" % (p[0], p[1], p[2]))

        node = config.getChild('ax', create=True)
        p = self.ax_bias
        node.setString('bias', "%.8f %.8f %.8f" % (p[0], p[1], p[2]))
        p = self.ax_scale
        node.setString('scale', "%.8f %.8f %.8f" % (p[0], p[1], p[2]))

        node = config.getChild('ay', create=True)
        p = self.ay_bias
        node.setString('bias', "%.8f %.8f %.8f" % (p[0], p[1], p[2]))
        p = self.ay_scale
        node.setString('scale', "%.8f %.8f %.8f" % (p[0], p[1], p[2]))

        node = config.getChild('az', create=True)
        p = self.az_bias
        node.setString('bias', "%.8f %.8f %.8f" % (p[0], p[1], p[2]))
        p = self.az_scale
        node.setString('scale', "%.8f %.8f %.8f" % (p[0], p[1], p[2]))

        affine_str = []
        for x in self.mag_affine.flat:
            affine_str.append('%.10f' % x)
        print ' '.join(affine_str)
        config.setString('mag_affine', ' '.join(affine_str))
        
        try:
            props_json.save(cal_file, config)
        except:
            print "error saving " + cal_file + ": " + str(sys.exc_info()[1])
            return

    # correct the IMU data given the current bias and scale errors
    def correct(self, imu_data):
        imu_corrected = []
        p_bias_func = np.poly1d(self.p_bias)
        q_bias_func = np.poly1d(self.q_bias)
        r_bias_func = np.poly1d(self.r_bias)
        ax_bias_func = np.poly1d(self.ax_bias)
        ay_bias_func = np.poly1d(self.ay_bias)
        az_bias_func = np.poly1d(self.az_bias)
        p_scale_func = np.poly1d(self.p_scale)
        q_scale_func = np.poly1d(self.q_scale)
        r_scale_func = np.poly1d(self.r_scale)
        ax_scale_func = np.poly1d(self.ax_scale)
        ay_scale_func = np.poly1d(self.ay_scale)
        az_scale_func = np.poly1d(self.az_scale)
        for imu in imu_data:
            newimu = nav.structs.IMUdata()
            newimu.time = imu.time
            temp = imu.temp
            newimu.temp = temp
            #newimu.status = imu.status
            if temp < self.min_temp:
                temp = self.min_temp
            if temp > self.max_temp:
                temp = self.max_temp    
            newimu.p = (imu.p - p_bias_func(temp)) * p_scale_func(temp)
            newimu.q = (imu.q - q_bias_func(temp)) * q_scale_func(temp)
            newimu.r = (imu.r - r_bias_func(temp)) * r_scale_func(temp)
            newimu.ax = (imu.ax - ax_bias_func(temp)) * ax_scale_func(temp)
            newimu.ay = (imu.ay - ay_bias_func(temp)) * ay_scale_func(temp)
            newimu.az = (imu.az - az_bias_func(temp)) * az_scale_func(temp)
            hs = [imu.hx, imu.hy, imu.hz, 1.0]
            hf = np.dot(self.mag_affine, hs)
            norm = np.linalg.norm(hf[:3])
            #hf[:3] /= norm
            newimu.hx = hf[0]
            newimu.hy = hf[1]
            newimu.hz = hf[2]
            imu_corrected.append(newimu)
        return imu_corrected
    
    # back correct the IMU data given the current bias and scale errors
    # (i.e. assuming corrected data, generate the raw values)
    def back_correct(self, imu_data):
        if not self.valid:
            return imu_data
        
        imu_corrected = []
        p_bias_func = np.poly1d(self.p_bias)
        q_bias_func = np.poly1d(self.q_bias)
        r_bias_func = np.poly1d(self.r_bias)
        ax_bias_func = np.poly1d(self.ax_bias)
        ay_bias_func = np.poly1d(self.ay_bias)
        az_bias_func = np.poly1d(self.az_bias)
        p_scale_func = np.poly1d(self.p_scale)
        q_scale_func = np.poly1d(self.q_scale)
        r_scale_func = np.poly1d(self.r_scale)
        ax_scale_func = np.poly1d(self.ax_scale)
        ay_scale_func = np.poly1d(self.ay_scale)
        az_scale_func = np.poly1d(self.az_scale)
        for imu in imu_data:
            newimu = nav.structs.IMUdata()
            newimu.time = imu.time
            temp = imu.temp
            newimu.temp = temp
            #newimu.status = imu.status
            if temp < self.min_temp:
                temp = self.min_temp
            if temp > self.max_temp:
                temp = self.max_temp    
            newimu.p = imu.p / p_scale_func(temp) + p_bias_func(temp)
            newimu.q = imu.q / q_scale_func(temp) + q_bias_func(temp)
            newimu.r = imu.r / r_scale_func(temp) + r_bias_func(temp)
            newimu.ax = imu.ax / ax_scale_func(temp) + ax_bias_func(temp)
            newimu.ay = imu.ay / ay_scale_func(temp) + ay_bias_func(temp)
            newimu.az = imu.az / az_scale_func(temp) + az_bias_func(temp)
            # note: corrected mags are currently being logged so don't
            # back correct mags here...
            back_correct_mags = False
            if back_correct_mags:
                hs = [imu.hx, imu.hy, imu.hz, 1.0]
                hf = np.dot(self.mag_affine_inv, hs)
                norm = np.linalg.norm(hf[:3])
                # #hf[:3] /= norm
                newimu.hx = hf[0]
                newimu.hy = hf[1]
                newimu.hz = hf[2]
            else:
                newimu.hx = imu.hx
                newimu.hy = imu.hy
                newimu.hz = imu.hz
            #print 'orig:', newimu.az, 'cooked:', imu.az
            imu_corrected.append(newimu)
        return imu_corrected
