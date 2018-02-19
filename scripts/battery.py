class linearfit:
    def __init__(self, time_factor, dt):
        self.time_factor = time_factor
        self.dt = dt
        # internal terms
        self.n = 0
        self.sum_x = 0.0
        self.sum_y = 0.0
        self.sum_x2 = 0.0
        self.sum_y2 = 0.0
        self.sum_xy = 0.0
        # the current fit estimate
        self.a0 = 0.0
        self.a1 = 0.0

    def update(self, x, y):
        max_n = self.time_factor / self.dt
        self.n += 1
        wf = 1.0
        if self.n > max_n:
            self.n = max_n
            wf = (float(self.n) - 1.0) / float(self.n)
    
        self.sum_x = self.sum_x * wf + x
        self.sum_y = self.sum_y * wf + y
        self.sum_x2 = self.sum_x2 * wf + x*x
        self.sum_y2 = self.sum_y2 * wf + y*y
        self.sum_xy = self.sum_xy * wf + x*y
        
        denom = float(self.n)*self.sum_x2 - self.sum_x*self.sum_x;
        #print self.sum_x, self.sum_y, self.sum_x2, self.sum_y2, self.sum_xy, denom
        if self.n > 1 and abs(denom) > 0.00001:
            self.a1 = (float(self.n)*self.sum_xy - self.sum_x*self.sum_y) / denom
            self.a0 = (self.sum_y - self.a1*self.sum_x) / float(self.n);
        else:
            self.a1 = 0.0
            self.a0 = y
        #print wf, self.a0, self.a1

    def get_value(self, x):
        return self.a1*x + self.a0;

class battery:
    def __init__(self, time_factor, dt):
        self.sag_model = linearfit(time_factor, dt)
        self.decay_model = linearfit(time_factor*2.0, dt)
        self.cutoff_vcc = 3.2

    def update(self, throttle, main_vcc, timestamp ):
        self.sag_model.update( throttle, main_vcc )
        full_vcc = self.sag_model.get_value( 1.0 )
        #print "t: %.1f thr: %.2f vcc: %.1f sag: %.1f" % (timestamp, throttle, main_vcc, full_vcc)
        self.decay_model.update( timestamp, full_vcc )
        if abs(self.decay_model.a1) > 0.00001:
            empty_t = (self.cutoff_vcc - self.decay_model.a0) / self.decay_model.a1
            #print " empty t:", empty_t
