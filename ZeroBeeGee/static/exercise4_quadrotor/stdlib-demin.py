Sk.builtinFiles={"files": {"src/lib/simulator/simulator.py": "import sys
import math
import numpy as np  # for cross, inv, random, arange
#from simulator.navdata import Navdata

if(sys.platform != \"skulpt\"):
    import matplotlib.pyplot as plt
    from pylab import *
    from mpl_toolkits.mplot3d import Axes3D
    import time

class Simulator():

    start_time = 0  # in secs
    end_time   = 1
    dt         = 0.005


    def __init__(self, drone, controller):
        self.drone         = drone
        self.controller    = controller
        self.step_count    = 0
        if(sys.platform != \"skulpt\"):
            self.x         = []
            self.y         = []
            self.z         = []
            self.roll      = []
            self.pitch     = []
            self.yaw       = []
            self.cmd1      = []
            self.cmd2      = []
            self.cmd3      = []
            self.cmd4      = []
            self.e_yaw     = []
            self.e_x       = []
            self.e_y       = []
            self.e_z       = []
            self.roll_des  = []
            self.pitch_des = []
            self.yaw_des   = []


    def reset(self):
        # TODO: reset all states
        self.theta_desired    = np.array([[0.0], [0.0], [0.0]])
        self.thetadot_desired = np.array([[0.0], [0.0], [0.0]])
        self.x_desired        = np.array([[0.0], [0.0], [0.0]])
        self.xdot_desired     = np.array([[0.0], [0.0], [0.0]])
        self.drone.x          = np.array([[0.0],[0.0],[0.0]])
        self.drone.xdot       = np.array([[0.0],[0.0],[0.0]])
        self.drone.xdoubledot = np.array([[0.0],[0.0],[0.0]])
        self.drone.theta      = np.array([[0.0],[0.0],[0.0]])
        self.drone.omega      = np.array([[0.0],[0.0],[0.0]])
        self.drone.thetadot   = np.array([[0.0],[0.0],[0.0]])

    def get_drone_pose(self):
        return [self.drone.x.item(0), self.drone.x.item(1), self.drone.x.item(2), self.drone.theta.item(0), self.drone.theta.item(1), self.drone.theta.item(2)];

    def get_drone_navdata(self):
        import simulator.navdata
        navdata=simulator.navdata.Navdata()

        local_velocity = np.dot(self.drone.yaw_rotation().transpose(), self.drone.xdot);

        navdata.vx   = local_velocity.item(0)
        navdata.vy   = local_velocity.item(1)
        navdata.vz   = local_velocity.item(2)
        navdata.ax   = self.drone.xdoubledot.item(0)
        navdata.ay   = self.drone.xdoubledot.item(1)
        navdata.az   = self.drone.xdoubledot.item(2)
        navdata.altd = self.drone.x.item(2)
        navdata.rotX = self.drone.theta.item(0)
        navdata.rotY = self.drone.theta.item(1)
        navdata.rotZ = self.drone.theta.item(2)
        return navdata;

    def set_input(self, sim_input):
        #self.theta_desired[0]   = sim_input[0];
        #self.theta_desired[1]   = sim_input[1];
        #self.theta_desired[2]   = sim_input[2];
        #self.xdot_desired[2]    = sim_input[3];
        self.xdot_desired[0]     = sim_input[0];
        self.xdot_desired[1]     = sim_input[1];
        self.thetadot_desired[2] = sim_input[2];
        self.xdot_desired[2]     = sim_input[3];
        self.xdot_desired        = np.dot(self.drone.yaw_rotation(), self.xdot_desired)

    def set_input_world(self, lin_vel, yaw_vel):
        self.xdot_desired[0]     = lin_vel[0]
        self.xdot_desired[1]     = lin_vel[1]
        self.xdot_desired[2]     = lin_vel[2]
        self.thetadot_desired[2] = yaw_vel;

    def simulate_step(self, t, dt):
        self.step_count += 1

        #if(int(t / 8.0) % 2 == 0):
        #    self.xdot_desired[0] = 0.75;
        #    self.xdot_desired[1] = 0.1;
        #else:
        #    self.xdot_desired[0] = -0.75;
        #    self.xdot_desired[1] = -0.1;
        #self.xdot_desired = np.dot(self.drone.yaw_rotation(), self.xdot_desired)


        #inputCurrents = self.controller.calculate_control_command(dt, self.theta_desired, self.thetadot_desired,self.x_desired,self.xdot_desired)
        inputCurrents, acc = self.controller.calculate_control_command3(dt, self.xdot_desired, self.thetadot_desired.item(2))
        omega = self.drone.omega;#thetadot_in_body()  # calculate current angular velocity in body frame

        #torques_thrust      = self.drone.torques_thrust(np.array([inputCurrents]).transpose())
        torques_thrust       = self.drone.torques_thrust(inputCurrents)
        #print acc.transpose();
        # print omega.transpose()
        linear_acceleration  = self.linear_acceleration(torques_thrust[3], self.drone.theta, self.drone.xdot)  # calculate the resulting linear acceleration
        omegadot             = self.angular_acceleration(torques_thrust[0:3,0], omega)  # calculate resulting angular acceleration
        #print \"angular acc\", omegadot.transpose()
        #linear_acceleration = self.linear_acceleration2(inputCurrents, self.drone.theta, self.drone.xdot)  # calculate the resulting linear acceleration
        #omegadot            = self.angular_acceleration2(inputCurrents, omega)  # calculate resulting angular acceleration

        omega = omega + dt * omegadot  # integrate up new angular velocity in the body frame

        omega[0] = math.atan2(math.sin(omega[0]), math.cos(omega[0])) #normalize roll, pitch, yaw
        omega[1] = math.atan2(math.sin(omega[1]), math.cos(omega[1]))
        omega[2] = math.atan2(math.sin(omega[2]), math.cos(omega[2]))

        #print \"inputs:\", inputCurrents
        #print \"omega:\", omega.transpose(), \"omegadot:\", omegadot.transpose()
        self.drone.omega    = omega;
        self.drone.thetadot = np.dot(self.drone.angle_rotation_to_world().transpose(), omega)  # calculate roll, pitch, yaw velocities
        self.drone.theta    = self.drone.theta + dt * self.drone.thetadot  # calculate new roll, pitch, yaw angles

        #print self.drone.xdot.transpose(), self.drone.theta.transpose(), omega.transpose(), omegadot.transpose()
        #print \"d\", inputCurrents
        #print \"a\", torques_thrust[0:3,0], \"b\", omega, \"c\", omegadot

        #print \"thetadot:\",self.drone.thetadot
        #print(\"New theta\",self.drone.theta)
        self.drone.xdoubledot = linear_acceleration
        self.drone.xdot       = self.drone.xdot + dt * linear_acceleration  # calculate new linear drone speed
        self.drone.x          = self.drone.x + dt * self.drone.xdot  # calculate new drone position
        #print \"acc\", linear_acceleration
        #print \"theta\", self.drone.theta
        #print(\"Position\",self.drone.x.transpose())

        if(sys.platform == \"skulpt\"):
            import plot
            plot.plot_pose(\"ardrone\", self.drone.x, self.drone.theta)
            plot.plot_trajectory(\"ardrone\", self.drone.x)
            plot.plot_motor_command(\"ardrone\", inputCurrents)


        elif(self.step_count % 50 == 0): #save trajectory for plotting
            vel  = np.dot(self.rotation(self.drone.theta).transpose(), self.drone.xdot)
            vel  = self.drone.xdot
            #vel = self.drone.x
            #vel = acc
            #vel = self.drone.thetadot_in_body();
            #vel = self.drone.xdot;

            self.x.append(vel.item(0))
            self.y.append(vel.item(1))
            self.z.append(vel.item(2))

            ang = self.drone.theta

            self.roll.append( ang.item(0))
            self.pitch.append(ang.item(1))
            self.yaw.append(  ang.item(2))
            #print self.theta_desired.item(2)
            self.cmd1.append(inputCurrents[0] - inputCurrents[2])
            self.cmd2.append(inputCurrents[1] - inputCurrents[3])
            self.cmd3.append(inputCurrents[0] - inputCurrents[1])
            self.cmd4.append(inputCurrents[3])
            self.roll_des.append(self.theta_desired[0])
            self.pitch_des.append(self.theta_desired[1])
            self.yaw_des.append(self.theta_desired[2])



    def simulate(self, duration):
        self.end_time = duration
        self.reset()

        # Step through the simulation, updating the drone state.
        t    = self.start_time
        fig1 = figure(1)
        fig2 = figure(2)
        fig3 = figure(3)
        fig4 = figure(4)


        while t <= self.end_time:
            self.simulate_step(t, self.dt)
            t += self.dt

            # only plot every
            frac = 5.0 * t + self.dt * 0.1;

            if((frac - int(frac)) < (self.dt * 0.5) and sys.platform != \"skulpt\"):
                #ion()
                ###########################################
                plt.figure(1)
                fig1.suptitle('Position x,y,z')
                #ax  = fig1.add_subplot(111, projection='3d')
                #ax.plot(self.x, self.y, self.z)
                #ax.axis([-5, 5, -5, 5])
                #ax.set_zlim3d( -5, 5 )
                #ax.set_xlabel('x')
                #ax.set_ylabel('y')
                #ax.set_zlabel('z')
                #plt.ylim(-1.5,+1.5)
                #plt.xlim(-1.5,+1.5)
                ax_x = fig1.add_subplot(311)
                ax_y = fig1.add_subplot(312)
                ax_z = fig1.add_subplot(313)
                #ax_z.ylim(-2.0, 2)
                ax_x.plot(self.x)
                ax_y.plot(self.y)
                ax_z.plot(self.z)
                draw()
                fig1.show()

                ###########################################
                plt.figure(2)
                fig2.suptitle('Position roll, pitch, yaw')
                ax_roll  = fig2.add_subplot(311)
                ax_roll.plot(self.roll)
                ax_roll.plot(self.roll_des)
                #ax_roll.legend(\"des\",\"act\",right)
                ax_pitch = fig2.add_subplot(312, sharey=ax_roll)
                ax_pitch.plot(self.pitch)
                ax_pitch.plot(self.pitch_des)

                ax_yaw = fig2.add_subplot(313, sharey=ax_roll)
                ax_yaw.plot(self.yaw)
                ax_yaw.plot(self.yaw_des)
                draw()
                fig2.show()

                ###########################################
                #plt.figure(3)
                #plt.ylim(-5,+5)
                #fig3.suptitle('Errors x,y,z,yaw ')
                #ax_x   = fig3.add_subplot(411)
                #ax_x.plot(self.e_x)
                #ax_y   = fig3.add_subplot(412, sharey=ax_x)
                #ax_y.plot(self.e_y)
                #ax_z   = fig3.add_subplot(413, sharey=ax_x)
                #ax_z.plot(self.e_z)
                #ax_yaw = fig3.add_subplot(414, sharey=ax_x)
                #ax_yaw.plot(self.e_yaw)
                #draw()
                #fig3.show()
                ############################################
                plt.figure(4)
                #plt.ylim(-2,2)
                fig4.suptitle('Control Commands')
                ax_1    = fig4.add_subplot(411)
                ax_1.plot(self.cmd1)
                ax_2    = fig4.add_subplot(412,sharey=ax_1)
                ax_2.plot(self.cmd2)
                ax_3    = fig4.add_subplot(413,sharey=ax_1)
                ax_3.plot(self.cmd3)
                #ax_4   = fig4.add_subplot(414,sharey=ax_1)
                #ax_4.plot(self.cmd4)
                fig4.show()
                #pause(0.1)

    def deg2rad(self,degrees):
        return np.array(map(math.radians, degrees))

    def rotation(self, angles):  # translate angles to intertial/world frame
        phi   = angles.item(0)
        theta = angles.item(1)
        psi   = angles.item(2)

        c_phi   = math.cos(phi);
        s_phi   = math.sin(phi);
        c_theta = math.cos(theta);
        s_theta = math.sin(theta);
        c_psi   = math.cos(psi)
        s_psi   = math.sin(psi)

        #ZYZ Euler nach Paper
        #R = np.array([[c_phi * c_psi - c_theta * s_phi * s_psi, -c_psi * s_phi - c_phi * c_theta * s_psi, s_theta * s_psi],
        #              [c_theta * c_psi * s_phi + c_phi * s_psi, c_phi * c_theta * c_psi - s_phi * s_psi,  -c_psi * s_theta],
        #              [s_phi * s_theta, c_phi * s_theta, c_theta]])
        # Master thesis XYZ
        R = np.array([[c_psi * c_theta, c_psi * s_theta * s_phi - s_psi * c_phi, c_psi * s_theta * c_phi + s_psi * s_phi],
                      [s_psi * c_theta, s_psi * s_theta * s_phi + c_psi * c_phi, s_psi * s_theta * c_phi - c_psi * s_phi],
                      [-s_theta, c_theta * s_phi, c_theta * c_phi]])

        #ZYZ Euler nach craig
        #R = np.array([[math.cos(psi)*math.cos(theta)*math.cos(phi)-math.sin(psi)*math.sin(phi), -math.cos(psi)*math.cos(theta)*math.sin(phi)-math.sin(psi)*math.cos(phi), math.cos(psi)*math.sin(theta) ],
        #              [math.sin(psi)*math.cos(theta)*math.cos(phi)+math.cos(psi)*math.sin(phi), -math.sin(psi)*math.cos(theta)*math.sin(phi)+math.cos(psi)*math.cos(phi), math.sin(psi)*math.sin(theta) ],
        #              [-math.sin(theta)*math.cos(phi), math.sin(theta)*math.sin(phi), math.cos(theta)]])

        return R

    def linear_acceleration(self, thrust, angles, xdot):
        gravity = np.array([[0], [0], [-self.drone.g]])
        R = self.rotation(angles)

        T      = np.dot(R, np.array([[0], [0], [thrust]]))
        F_drag = -self.drone.kd * xdot
        a      = gravity + (T + F_drag) / self.drone.m
        return a

    def angular_acceleration(self, torques, omega):
        # this transpose stuff really sucks
        omegaddot = np.dot(self.drone.I_inv, (torques.transpose() - np.cross(omega.transpose(), np.dot(self.drone.I, omega).transpose())).transpose());
        return omegaddot
", "src/lib/time/__init__.js": "
/*
	Barebones implementation of the Python time package.

	For now, only the time() function is implemented.
*/
 
var $builtinmodule = function(name)
{
    var mod = {};

    mod.time = new Sk.builtin.func(function() {
	  return Sk.builtin.assk$(new Date().getTime() / 1000, undefined);
    });

    return mod;
}
", "src/lib/numpy/backup_init__.js": "var $builtinmodule = function(name)
{
  var mod = {};
  
  /**
   * ndarray class
   */
  var ndarray = function($gbl, $loc) {
    $loc.__init__ = new Sk.builtin.func(function(self, shape, buffer) {
      Sk.builtin.pyCheckArgs('numpy.ndarray.__init__', arguments, 2, 3);
      
      self.$d.mp$ass_subscript(new Sk.builtin.str('shape'), shape);
      self.$d.mp$ass_subscript(new Sk.builtin.str('ndim'), new Sk.builtin.nmber(shape.v.length, Sk.builtin.nmber.int$));
      self.data = buffer;
    });
  };
  
  mod.ndarray = Sk.misceval.buildClass(mod, ndarray, 'ndarray', []);
  
  function compare(arr1, arr2) {
    if(arr1.length != arr2.length) return false;
    
    var idx;
    for(idx = 0; idx < arr1.length; idx++) {
      if(arr1[idx] != arr2[idx]) return false;
    }
    
    return true;
  }
  
  function getShapeFromList(l) {
    if(l.tp$name !== undefined && l.tp$name === 'list') {
      if(l.v.length == 0) {
        return [0]
      }
    
      var shape = getShapeFromList(l.v[0]);
    
      var idx;
      for(idx = 1; idx < l.v.length; ++idx) {
        if(!compare(shape, getShapeFromList(l.v[idx]))) {
          console.log(\"inner dimensions, don't agree\");
          // TODO: error
        }
      }
      
      return [l.v.length].concat(shape);
    } else if(l.skType !== undefined) {
      return [];
    }
    
    // TODO: error
    console.log(\"no array, nor number\");
  }  
  
  /**
   * array creation functions
   */
  mod.array = new Sk.builtin.func(function(object) {
    Sk.builtin.pyCheckArgs('array', arguments, 1);
    
    return Sk.misceval.callsim(mod.ndarray, Sk.builtin.tuple(getShapeFromList(object)), object);
  });
  
  return mod;
}
", "src/lib/operator/__init__.js": "/*
 *  __author__: Isaac Dontje Lindell (i@isaacdontjelindell.com)
 *
 *  Implementation of the Python operator module.
 */

var $builtinmodule = function(name) {
    var mod = {};

    mod.lt = new Sk.builtin.func(function(a, b) { return Sk.builtin.bool(Sk.misceval.richCompareBool(a, b, 'Lt')); });
    mod.__lt__ = mod.lt;

    mod.le = new Sk.builtin.func(function(a, b) { return Sk.builtin.bool(Sk.misceval.richCompareBool(a, b, 'LtE')); });
    mod.__le__ = mod.le;

    mod.eq = new Sk.builtin.func(function(a, b) { return Sk.builtin.bool(Sk.misceval.richCompareBool(a, b, 'Eq')); });
    mod.__eq__ = mod.eq;

    mod.ne = new Sk.builtin.func(function(a, b) { return Sk.builtin.bool(Sk.misceval.richCompareBool(a, b, 'NotEq')); });
    mod.__ne__ = mod.ne;

    mod.ge = new Sk.builtin.func(function(a, b) { return Sk.builtin.bool(Sk.misceval.richCompareBool(a, b, 'GtE')); });
    mod.__ge__ = mod.ge;

    mod.gt = new Sk.builtin.func(function(a, b) { return Sk.builtin.bool(Sk.misceval.richCompareBool(a, b, 'Gt')); });
    mod.__gt__ = mod.gt;

    mod.not_ = new Sk.builtin.func(function (obj) { throw new Sk.builtin.NotImplementedError(\"operator.not_() is not yet implemented in Skulpt\"); });

    mod.truth = new Sk.builtin.func(function(obj) { return Sk.builtin.bool(obj); });

    mod.is_ = new Sk.builtin.func(function(a, b) { return Sk.builtin.bool(Sk.misceval.richCompareBool(a, b, 'Is')); });

    mod.is_not = new Sk.builtin.func(function(a, b) { return Sk.builtin.bool(Sk.misceval.richCompareBool(a, b, 'IsNot')); });

    mod.abs = new Sk.builtin.func(function(obj) { return Sk.misceval.callsim(Sk.builtin.abs, obj); });
    mod.__abs__ = mod.abs;

    // The documentation says that operator.add() is defined for a and b numbers, but
    // CPython (2.6) allows a and b to be other types (e.g. str)
    mod.add = new Sk.builtin.func(function (a, b) { return Sk.abstr.objectAdd(a, b); });
    mod.__add__ = mod.add;

    mod.and_ = new Sk.builtin.func(function (a, b) { return Sk.builtin.nmber.prototype['nb$and'].call(a, b); });
    mod.__and__ = mod.and_;

    mod.div = new Sk.builtin.func(function (a, b) { return Sk.builtin.nmber.prototype['nb$divide'].call(a, b); });
    mod.__div__ = mod.div;

    mod.floordiv = new Sk.builtin.func(function (a, b) { return Sk.builtin.nmber.prototype['nb$floor_divide'].call(a, b); });
    mod.__floordiv__ = mod.floordiv;

    // Doesn't look like anything has the __index__ magic function anyway
    mod.index = new Sk.builtin.func(function (a) { throw new Sk.builtin.NotImplementedError(\"operator.index() is not yet implemented in Skulpt\"); });
    mod.__index__ = mod.index;

    // Note: Sk.abstr.numberUnaryOp(obj, 'Invert') looks for the function nb$invert() on obj.
    // However, it doesn't look like that function has been implemented for any existing object types.
    // I've gone ahead and created this function for completeness' sake, but expect any use of it to
    // result in an error.
    mod.inv = new Sk.builtin.func(function (obj) { return Sk.abstr.numberUnaryOp(obj, 'Invert'); });
    mod.__inv__ = mod.inv;
    mod.invert = mod.inv;
    mod.__invert__ = mod.inv;

    mod.lshift = new Sk.builtin.func(function (a, b) { return Sk.builtin.nmber.prototype['nb$lshift'].call(a, b); });
    mod.__lshift__ = mod.lshift;

    mod.mod = new Sk.builtin.func(function (a, b) { return Sk.builtin.nmber.prototype['nb$remainder'].call(a, b); });
    mod.__mod__ = mod.mod;

    mod.mul = new Sk.builtin.func(function (a, b) { return Sk.builtin.nmber.prototype['nb$multiply'].call(a, b); });
    mod.__mul__ = mod.mul;

    mod.neg = new Sk.builtin.func(function (obj) { return Sk.abstr.objectNegative(obj); });
    mod.__neg__ = mod.neg;

    mod.or_ = new Sk.builtin.func(function (a, b) { return Sk.builtin.nmber.prototype['nb$or'].call(a, b); });
    mod.__or__ = mod.or_;

    mod.pos = new Sk.builtin.func(function (obj) { return Sk.abstr.objectPositive(obj); });
    mod.__pos__ = mod.pos;

    mod.pow = new Sk.builtin.func(function (a, b) { return Sk.builtin.nmber.prototype['nb$power'].call(a, b); });
    mod.__pow__ = mod.pow;

    mod.rshift = new Sk.builtin.func(function (a, b) { return Sk.builtin.nmber.prototype['nb$rshift'].call(a, b); });
    mod.__rshift__ = mod.rshift;

    mod.sub = new Sk.builtin.func(function (a, b) { return Sk.builtin.nmber.prototype['nb$subtract'].call(a, b); });
    mod.__sub__ = mod.sub;

    mod.truediv = mod.div;
    mod.__truediv__ = mod.div;

    mod.xor = new Sk.builtin.func(function (a, b) { return Sk.builtin.nmber.prototype['nb$xor'].call(a, b); });
    mod.__xor__ = mod.xor;

    mod.concat = new Sk.builtin.func(function (a, b) { return Sk.abstr.sequenceConcat(a, b); });
    mod.__concat__ = mod.concat;

    mod.contains = new Sk.builtin.func(function (a, b) { return Sk.builtin.bool(Sk.abstr.sequenceContains(a, b)); });
    mod.__contains__ = mod.contains;

    mod.countOf = new Sk.builtin.func(function (a, b) { return Sk.abstr.sequenceGetCountOf(a, b); });

    mod.delitem = new Sk.builtin.func(function (a, b) { return Sk.abstr.sequenceDelItem(a, b); });
    mod.__delitem__ = mod.delitem;

    mod.getitem = new Sk.builtin.func(function (a, b) { return Sk.abstr.sequenceGetItem(a, b); });
    mod.__getitem__ = mod.getitem;

    mod.indexOf = new Sk.builtin.func(function (a, b) { return Sk.abstr.sequenceGetIndexOf(a, b); });

    mod.setitem = new Sk.builtin.func(function (a, b, c) { return Sk.abstr.sequenceSetItem(a, b, c); });
    mod.__setitem__ = mod.setitem;

    return mod;
};
", "src/lib/simulator/setup.py": "'''
Created on 07.02.2014

@author: tatsch
'''

def setup():
    from simulator.controller import Controller
    from simulator.drone import Drone
    from simulator.simulator import Simulator
    import numpy as np;
    
    drone = Drone()
    #angular_disturbance = np.array([[0.03], [0.02], [0.1]])
    #drone.thetadot = angular_disturbance  # Simulate some disturbance in the angular velocity.
    controller = Controller(drone)
    simulator = Simulator(drone,controller)
    
    return simulator;
", "src/lib/unittest/__init__.py": "__author__ = 'bmiller'
'''
This is the start of something that behaves like
the unittest module from cpython.

'''


class TestCase:
    def __init__(self):
        self.numPassed = 0
        self.numFailed = 0

        self.tlist = []
        testNames = {}
        for name in dir(self):
            if name[:4] == 'test' and name not in testNames:
                self.tlist.append(getattr(self,name))
                testNames[name]=True

    def setup(self):
        pass

    def tearDown(self):
        pass

    def main(self):

        for func in self.tlist:
            try:
                self.setup()
                func()
                self.tearDown()
            except:
                self.appendResult('Error',None,None,None)
                self.numFailed += 1
        self.showSummary()

    def assertEqual(self, actual, expected, feedback=\"\"):
        res = actual==expected
        self.appendResult(res,str(actual)+' to be equal to ',expected, feedback)

    def assertNotEqual(actual, expected, feedback=\"\"):
        res = actual != expected
        self.appendResult(res,str(actual)+' to not equal ',expected,feedback)

    def assertTrue(self,x, feedback=\"\"):
        res = x
        self.appendResult(res,str(x)+' to be ',True,feedback)

    def assertFalse(self,x, feedback=\"\"):
        res = not x
        self.appendResult(res,str(x)+' to be ',False,feedback)

    def assertIs(self,a,b, feedback=\"\"):
        res = a is b
        self.appendResult(res,str(a)+' to be the same object as ',b,feedback)

    def assertIsNot(self,a,b, feedback=\"\"):
        res = a is not b
        self.appendResult(res,str(a)+' to not be the same object as ',b,feedback)

    def assertIsNone(self,x, feedback=\"\"):
        res = x is None
        self.appendResult(res,x,None,feedback)

    def assertIsNotNone(self,x, feedback=\"\"):
        res = x is not None
        self.appendResult(res,str(x)+' to not be ',None,feedback)

    def assertIn(self,a,b, feedback=\"\"):
        res = a in b
        self.appendResult(res,str(a)+' to be in ',b,feedback)

    def assertNotIn(self,a,b, feedback=\"\"):
        res = a not in b
        self.appendResult(res,str(a)+' to not be in ',b,feedback)

    def assertIsInstance(self,a,b, feedback=\"\"):
        res = isinstance(a,b)
        self.appendResult(res,str(a)+' to be an instance of ',b,feedback)

    def assertNotIsInstance(self,a,b, feedback=\"\"):
        res = not isinstance(a,b)
        self.appendResult(res,str(a)+' to not be an instance of ',b,feedback)

    def assertAlmostEqual(self,a,b, feedback=\"\"):
        res = round(a-b,7) == 0
        self.appendResult(res,str(a)+' to equal ',b,feedback)

    def assertNotAlmostEqual(self,a,b, feedback=\"\"):
        res = round(a-b,7) != 0
        self.appendResult(res,str(a)+' to not equal ',b,feedback)

    def assertGreater(self,a,b, feedback=\"\"):
        res = a > b
        self.appendResult(res,str(a)+' to be greater than ',b,feedback)

    def assertGreaterEqual(self,a,b, feedback=\"\"):
        res = a >= b
        self.appendResult(res,str(a)+' to be greater than or equal to ',b,feedback)

    def assertLess(self,a,b, feedback=\"\"):
        res = a < b
        self.appendResult(res,str(a)+' to be less than ',b,feedback)

    def assertLessEqual(self,a,b, feedback=\"\"):
        res = a <= b
        self.appendResult(res,str(a)+' to be less than or equal to ',b,feedback)

    def appendResult(self,res,actual,expected,feedback):
        if res == 'Error':
            msg = 'Error'
        elif res:
            msg = 'Pass'
            self.numPassed += 1
        else:
            msg = 'Fail: expected %s  %s ' % (str(actual),str(expected)) + feedback
            self.numFailed += 1

    def showSummary(self):
        pct = self.numPassed / (self.numPassed+self.numFailed) * 100
        print \"ran %d tests, passed %d \
\" % (self.numPassed+self.numFailed, self.numPassed)



def main():
    glob = globals()  # globals() still needs work
    for name in glob:
        if issubclass(glob[name],TestCase):
            glob[name]().main()  

", "src/lib/test/__init__.py": "__author__ = 'bmiller'

def testEqual(actual, expected):
    if type(expected) == type(1):
        if actual == expected:
            print('Pass')
            return True
    elif type(expected) == type(1.11):
        if abs(actual-expected) < 0.00001:
            print('Pass')
            return True
    else:
        if actual == expected:
            print('Pass')
            return True
    print('Test Failed: expected ' + str(expected) + ' but got ' + str(actual))
    return False

def testNotEqual(actual, expected):
    pass

", "src/lib/string/__init__.js": "/*
 *  __author__: Isaac Dontje Lindell (i@isaacdontjelindell.com)
 *
 *  Implementation of the Python string module.
 */


var $builtinmodule = function(name) {
    var mod = {};

    mod.ascii_lowercase = Sk.builtin.str('abcdefghijklmnopqrstuvwxyz');
    mod.ascii_uppercase = Sk.builtin.str('ABCDEFGHIJKLMNOPQRSTUVWXYZ');
    mod.ascii_letters = Sk.builtin.str(mod.ascii_lowercase.v + mod.ascii_uppercase.v);

    mod.lowercase = Sk.builtin.str('abcdefghijklmnopqrstuvwxyz');
    mod.uppercase = Sk.builtin.str('ABCDEFGHIJKLMNOPQRSTUVWXYZ');
    mod.letters = Sk.builtin.str(mod.lowercase.v + mod.uppercase.v);

    mod.digits = Sk.builtin.str('0123456789', Sk.builtin.str);
    mod.hexdigits = Sk.builtin.str('0123456789abcdefABCDEF');
    mod.octdigits = Sk.builtin.str('01234567');

    mod.punctuation = Sk.builtin.str('!\"#$%&\\'()*+,-./:;<=>?@[\\\\]^_`{|}~');
    mod.whitespace = Sk.builtin.str('\	\
\\x0b\\x0c\\r ');

    /* Note: The docs for string.printable say that it's the concatenation of string.digits,
     * string.letters, string.punctuation, and string.whitespace. The CPython interpreter
     * outputs the whitespace characters in one order when string.whitespace is used, and a
     * slightly different order when string.printable is used. I've elected to follow the
     * behavior of CPython here rather than the spec. */
    mod.printable = Sk.builtin.str(mod.digits.v + mod.letters.v + mod.punctuation.v + \" \	\
\\r\\x0b\\x0c\");


    mod.split = new Sk.builtin.func(function(s, sep, maxsplit) {
        return Sk.misceval.callsim(Sk.builtin.str.prototype['split'], s, sep, maxsplit);
    });

    /* Return a copy of word with only its first character capitalized. */
    mod.capitalize = new Sk.builtin.func(function(word) {
        return Sk.misceval.callsim(Sk.builtin.str.prototype['capitalize'], word);
    });

    /* Concatenate a list or tuple of words with intervening occurrences
     * of sep. The default value for sep is a single space character. */
    mod.join = new Sk.builtin.func(function(words, sep) {
        if (sep === undefined) {
            sep = Sk.builtin.str(' ');
        }
        return Sk.misceval.callsim(Sk.builtin.str.prototype['join'], sep, words);
    });


    /* Split the argument into words using split(), capitalize each word
     * using capitalize(), and join the capitalized words using join().
     * Note that this replaces runs of whitespace characters by a single
     * space, and removes leading and trailing whitespace. */
    mod.capwords = new Sk.builtin.func(function(s, sep) {
        Sk.builtin.pyCheckArgs('capwords', arguments, 1, 2);
        if (!Sk.builtin.checkString(s)) {
            throw new Sk.builtin.TypeError(\"s must be a string\");
        }
        if (sep === undefined) {
            sep = Sk.builtin.str(' ');
        }
        if(!Sk.builtin.checkString(sep)) {
            throw new Sk.builtin.TypeError(\"sep must be a string\");
        }

        var words = Sk.misceval.callsim(mod.split, s, sep);
        var capWords = [];
        for (var i=0; i<words.v.length; i++) {
            var word = Sk.builtin.list.prototype['list_subscript_'].call(words, i);
            var cap = Sk.misceval.callsim(mod.capitalize, word);
            capWords.push(cap);
        }

        return Sk.misceval.callsim(mod.join, Sk.builtin.list(capWords), sep);
    });


    return mod;
};", "src/builtin/sys.js": "var $builtinmodule = function(name)
{
    var sys = {};

    var args = [];
    var argv = Sk.getSysArgv();
    for (var i = 0; i < argv.length; ++i)
        args.push(new Sk.builtin.str(argv[i]));
    sys.argv = new Sk.builtins['list'](args);

    sys.copyright = Sk.builtin['str'](\"Copyright 2009-2010 Scott Graham.\
All Rights Reserved.\
\");
    sys.platform = Sk.builtin['str'](\"skulpt\");


    sys.modules = Sk.sysmodules;

    sys.path = Sk.realsyspath;

    sys.getExecutionLimit = new Sk.builtin.func(function() {
        return Sk.builtin.assk$(Sk.execLimit, Sk.builtin.nmber.int$);
    });

    sys.setExecutionLimit = new Sk.builtin.func(function(t) {
        if (t !==  undefined) {
            Sk.execLimit = Sk.builtin.asnum$(t);
        }
    });

    sys.resetTimeout = new Sk.builtin.func(function() {
        Sk.execStart = new Date();
    });

    sys.debug = new Sk.builtin.func(function() {
        debugger;
    });

    return sys;
};
", "src/lib/quadrotor/command.py": "from simulator.controller import RelativeOrder

def forward(distance):
    return RelativeOrder(distance, 0, 0, 0)

def backward(distance):
    return RelativeOrder(-distance, 0, 0, 0)

def left(distance):
    return RelativeOrder(0, distance, 0, 0)

def right(distance):
    return RelativeOrder(0, -distance, 0, 0)

def up(distance):
    return RelativeOrder(0, 0, distance, 0)

def down(distance):
    return RelativeOrder(0, 0, -distance, 0)

def turn_left(angle):
    return RelativeOrder(0, 0, 0, angle)

def turn_right(angle):
    return RelativeOrder(0, 0, 0, -angle)
", "src/lib/simulator/__init__.py": "", "src/lib/simulator/main.py": "import numpy as np
from controller import Controller
from drone import Drone
from simulator import Simulator

def run():
    drone = Drone()
    controller = Controller(drone)
    simulator = Simulator(drone, controller)
    deviation = 1;   # Simulate some disturbance in the angular velocity.
    #angular_disturbance=simulator.deg2rad(2 * deviation * np.random.rand(3, 1) - deviation).transpose()
    angular_disturbance = np.array([[0.0],[0.1],[0.0]])
    #drone.thetadot = angular_disturbance  # Simulate some disturbance in the angular velocity.
    simulator.simulate(60)  # simulate n secs
    
    import matplotlib.pyplot
    matplotlib.pyplot.pause(60)
run()", "src/lib/plot/__init__.js": "var $builtinmodule = function(name)
{
  Sk.interop['plot'] = Sk.interop['plot'] || {
    'data': {
      'scalar': {},
      'pose': {},
      'motor_command': {},
      'trajectory': {},
      'points': {},
      'covariance2d': {},
      'covariance3d': {},
      'marker': {},
    }
  };

  var plot = Sk.interop['plot'];

  plot['clear'] = function() {
    Object.keys(plot.data).forEach(function(type) {
      //plot.data[type] = {};
      Object.keys(plot.data[type]).forEach(function(name) {
        plot.data[type][name] = [];
      });
    });
  };
  plot['reset'] = function() {
    Object.keys(plot.data).forEach(function(type) {
      plot.data[type] = {};
    });
  };

  var addPlotValue = function(type, name, value) {
    if (plot.data[type][name] == undefined) {
        plot.data[type][name] = [];
    }

    plot.data[type][name].push(value);
  };

  var mod = {};

  mod.plot = new Sk.builtin.func(function(name, value) {
    Sk.builtin.pyCheckArgs('plot', arguments, 2);
    Sk.builtin.pyCheckType('name', 'string', Sk.builtin.checkString(name));
    Sk.builtin.pyCheckType('value', 'number', Sk.builtin.checkNumber(value));

    addPlotValue('scalar', name.v, value.v);
  });

  mod.plot_pose = new Sk.builtin.func(function(name, position, orientation) {
    Sk.builtin.pyCheckArgs('plot_pose', arguments, 3);
    Sk.builtin.pyCheckType('name', 'string', Sk.builtin.checkString(name));

    addPlotValue('pose', name.v, [position.v.get([0, 0]), position.v.get([1, 0]), position.v.get([2, 0]), orientation.v.get([0, 0]), orientation.v.get([1, 0]), orientation.v.get([2, 0])]);
  });

  mod.plot_motor_command = new Sk.builtin.func(function(name, value) {
    Sk.builtin.pyCheckArgs('plot_motor_command', arguments, 2);
    Sk.builtin.pyCheckType('name', 'string', Sk.builtin.checkString(name));

    addPlotValue('motor_command', name.v, [value.v.get([0, 0]), value.v.get([1, 0]), value.v.get([2, 0]), value.v.get([3, 0])]);
  });

  mod.plot_trajectory = new Sk.builtin.func(function(name, position) {
    Sk.builtin.pyCheckArgs('plot_trajectory', arguments, 2);
    Sk.builtin.pyCheckType('name', 'string', Sk.builtin.checkString(name));

    var size = position.v.size();

    addPlotValue('trajectory', name.v, [position.v.get([0, 0]), position.v.get([1, 0]), size[0] > 2 ? position.v.get([2, 0]) : Infinity]);
  });

  mod.plot_point = new Sk.builtin.func(function(name, position) {
    Sk.builtin.pyCheckArgs('plot_point', arguments, 2);
    Sk.builtin.pyCheckType('name', 'string', Sk.builtin.checkString(name));

    var size = position.v.size();

    addPlotValue('points', name.v, [position.v.get([0, 0]), position.v.get([1, 0]), size[0] > 2 ? position.v.get([2, 0]) : 0]);
  });

  mod.plot_covariance_2d = new Sk.builtin.func(function(name, cov) {
    Sk.builtin.pyCheckArgs('plot_covariance_2d', arguments, 2, 2);
    Sk.builtin.pyCheckType('name', 'string', Sk.builtin.checkString(name));

    addPlotValue('covariance2d', name.v, cov.v.toArray());
  });

  mod.plot_covariance_3d = new Sk.builtin.func(function(name, cov) {
    Sk.builtin.pyCheckArgs('plot_covariance_3d', arguments, 2, 2);
    Sk.builtin.pyCheckType('name', 'string', Sk.builtin.checkString(name));

    addPlotValue('covariance3d', name.v, cov.v.toArray());
  });

  mod.plot_marker = new Sk.builtin.func(function(id, position, yaw) {
    Sk.builtin.pyCheckArgs('plot_marker', arguments, 3);

    addPlotValue('marker', id.v, [position.v.get([0, 0]), position.v.get([1, 0]), yaw.v]);
  });

  return mod;
};
", "src/lib/simulator/drone.py": "'''
Created on 07.02.2014

@author: tatsch
'''
import numpy as np

#Drone layout
#
#          1 O
#            |
#        x ^ |
#          | |
#            |          4
# O----------+----------O
# 2   <--    |
#      y     |
#            |
#            |
#            O 3

class Drone():
    '''
    Model the drone
    '''

    #As in \"Modeling Ardrone\"
    L = 0.18  # distance between prop and COG in m
    m = 436.5  # mass of the drone in g
    g = 980.66  # gm/s^2
    I = np.array([[2.04016E-2, 0, 0],
                  [0, 1.56771E-2, 0],
                  [0, 0, 3.51779E-2]])  # inertia matrix of the drone in gm3

    k_b = 3.29E-11 # drag coefficient Nm/rpm2
    k_t =  1.27E-7  # torque coefficient
    kd = 0.1  # air friction coefficent of the whole ardrone,

    #As in Hector Quadrotor Simulator
    #Psi:  0.007242179827506
    #J_M: 2.573048063300000e-5 #inertia Motor
    #R_A: 0.201084219222241 #resistance motor
    #k_t: 0.015336864714397 #m_m=k_t*T
    #k_m: -7.011631909766668e-5
    #alpha_m: 0.104863758313889
    #beta_m: 0.549262344777900
    #CT2s: -1.3077e-2 #Thrust koefficients k_t from a  quadratic model
    #CT1s: -2.5224e-4
    #CT0s:  1.538190483976698e-5
    #l_m: 0.275

    L = 0.18  # distance between prop and COG in m
    m = 0.4365  # mass of the drone in kg, for hovering 5.8 A
    g = 9.81  # gm/s^2
    I = np.array([[2.5730480633e-8, 0, 0],
                  [0, 2.573048063300000e-8, 0],
                  [0, 0, 2.573048063300000e-5]])  # inertia matrix of the drone in gm3
    I = np.array([[0.007, 0, 0],
                  [0, 0.007, 0],
                  [0, 0, 0.012]])  # inertia matrix of the drone in gm3
    k_b = 0.1 # drag coefficient Nm/rpm2
    k_t =  0.73  # thrust coefficient in g/
    kd = 0.12  # air friction coefficent of the whole ardrone,

    x = np.array([[0.0],
                  [0.0],
                  [0.0]])
    xdot = np.zeros((3, 1))
    xdoubledot = np.zeros((3, 1))

    theta = np.zeros((3, 1))
    theta_body = np.zeros((3, 1))
    thetadot = np.zeros((3, 1))
    thetadoubledot = np.zeros((3, 1))

    def __init__(self):
        self.I_inv = np.linalg.inv(self.I)

        # K matrix is diagonal containing constants
        # A matrix is allocation matrix describing configuration of quadrotor

        k = self.k_t
        kL = k * self.L
        b = self.k_b
        m = self.m
        Ixx = self.I.item((0, 0))
        Iyy = self.I.item((1, 1))
        Izz = self.I.item((2, 2))

        # matrix to compute torques/moments and thrust from motor inputs
        self.KA = np.array([[0.0,kL,0.0,-kL],[kL,0.0,-kL,0.0],[b,-b,b,-b],[k,k,k,k]])
        #self.KA = np.array([[0,-kL,0,kL],[kL,0,-kL,0],[b,-b,b,-b],[k,k,k,k]]);
        # matrix to compute motor inputs from desired angular acceleration and thrust
        self.AinvKinvI = np.array([[0.0,Iyy/(2.0*kL),Izz/(4.0*b),m/(4.0*k)],[Ixx/(2.0*kL),0.0,-Izz/(4.0*b),m/(4.0*k)],[0.0,-Iyy/(2.0*kL),Izz/(4.0*b),m/(4.0*k)],[-Ixx/(2.0*kL),0.0,-Izz/(4.0*b),m/(4.0*k)]])
        #self.AinvKinvI = np.array([[0,Iyy/(2*kL),Izz/(4*b),m/(4*k)],[-Ixx/(2*kL),0,-Izz/(4*b),m/(4*k)],[0,-Iyy/(2*kL),Izz/(4*b),m/(4*k)],[Ixx/(2*kL),0,-Izz/(4*b),m/(4*k)]]);


        # H configuration
        #self.KA = np.array([[kL,kL,-kL,-kL],[kL,-kL,-kL,kL],[-b,b,-b,b],[k,k,k,k]])
        #self.AinvKinvI = np.array([[Ixx/(4*kL),Iyy/(4*kL),-Izz/(4*b),m/(4*k)],[Ixx/(4*kL),-Iyy/(4*kL),Izz/(4*b),m/(4*k)],[-Ixx/(4*kL),-Iyy/(4*kL),-Izz/(4*b),m/(4*k)],[-Ixx/(4*kL),Iyy/(4*kL),Izz/(4*b),m/(4*k)]])

        self.K = np.array([[kL, 0, 0, 0],
                           [0, kL, 0, 0],
                           [0,  0, b, 0],
                           [0,  0, 0, k]])

        self.A = np.array([[ 1, 1,-1,-1],
                           [ 1,-1,-1, 1],
                           [-1, 1,-1, 1],
                           [ 1, 1, 1, 1]])

        tmp = np.array([[Ixx, 0, 0, 0],
                        [0, Iyy, 0, 0],
                        [0, 0, Izz, 0],
                        [0, 0, 0, m  ]])

        self.KA = np.dot(self.K, self.A);
        self.AinvKinvI = np.dot(np.dot(np.linalg.inv(self.A), np.linalg.inv(self.K)), tmp)

        # corke tutorial
        #self.KA = np.array([[0,kL,0,-kL],[-kL,0,kL,0],[-b,b,-b,b],[k,k,k,k]]);
        #self.AinvKinvI = np.array([[0,-Iyy/(2*kL),-Izz/(4*b),m/(4*k)],[Ixx/(2*kL),0,Izz/(4*b),m/(4*k)],[0,Iyy/(2*kL),-Izz/(4*b),m/(4*k)],[-Ixx/(2*kL),0,Izz/(4*b),m/(4*k)]]);

        pass

    def angle_rotation_to_body(self):
        '''
        compute rotation matrix to convert angular velocities to body frame
        '''
        from math import sin, cos

        phi = self.theta.item(0);
        theta = self.theta.item(1);

        #return np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

        return np.array([[1, 0, -sin(theta)],
                      [0, -cos(phi), cos(theta) * sin(phi)],
                      [0, sin(phi), cos(theta) * cos(phi)]])


    def yaw_rotation(self):
        '''
        compute rotation matrix to convert angular velocities to body frame
        '''
        from math import sin, cos

        psi = self.theta.item(2);
        cpsi = cos(psi)
        spsi = sin(psi)
        return np.array([[cpsi, -spsi, 0],
                      [spsi, cpsi, 0],
                      [0, 0, 1]])

    def angle_rotation_to_world(self):
        '''
        compute rotation matrix to convert angular velocities to world frame
        '''
        from math import sin, cos, tan, fabs

        phi = self.theta.item(0);
        theta = self.theta.item(1);
        #return np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

        return np.array([[1, sin(phi) * tan(theta), cos(phi) * tan(theta)],
                      [0, cos(phi), -sin(phi)],
                      [0, sin(phi) / cos(theta), cos(phi) / cos(theta)]])

    def theta_in_body(self):
        return np.dot(self.angle_rotation_to_body(), self.theta)

    def thetadot_in_body(self):
        return np.dot(self.angle_rotation_to_body(), self.thetadot)

    def torques_thrust(self, inputs):
        return np.dot(self.KA, inputs)

    # Compute motor torques, given the current input currents, length, drag coefficient, and thrust coefficients
    def torques(self, inputs):
        mu = np.array([[self.L * self.k_t, 0,                 0],
                       [0,              self.L * self.k_t,    0],
                       [0,              0,                  self.k_b]])
        inp = np.array([[inputs[0] - inputs[2]],
                        [inputs[1] - inputs[3]],
                        [inputs[0] - inputs[1] + inputs[2] - inputs[3]]])
        tau = np.dot(mu, inp)
        return tau

    def thrust(self, inputs):
        T = np.array([[0], [0], [self.k_t * sum(inputs)]])
        return T

    def rotation(self):  # translate angles to intertial/world frame
        import math
        phi = self.theta.item(0)
        theta = self.theta.item(1)
        psi = self.theta.item(2)

        c_phi = math.cos(phi);
        s_phi = math.sin(phi);
        c_theta = math.cos(theta);
        s_theta = math.sin(theta);
        c_psi = math.cos(psi)
        s_psi = math.sin(psi)

        #ZYZ Euler nach Paper
        #R = np.array([[c_phi * c_psi - c_theta * s_phi * s_psi, -c_psi * s_phi - c_phi * c_theta * s_psi, s_theta * s_psi],
        #              [c_theta * c_psi * s_phi + c_phi * s_psi, c_phi * c_theta * c_psi - s_phi * s_psi,  -c_psi * s_theta],
        #              [s_phi * s_theta, c_phi * s_theta, c_theta]])
        # Master thesis XYZ
        R = np.array([[c_psi * c_theta, c_psi * s_theta * s_phi - s_psi * c_phi, c_psi * s_theta * c_phi + s_psi * s_phi],
                      [s_psi * c_theta, s_psi * s_theta * s_phi + c_psi * c_phi, s_psi * s_theta * c_phi - c_psi * s_phi],
                      [-s_theta, c_theta * s_phi, c_theta * c_phi]])

        #ZYZ Euler nach craig
        #R = np.array([[math.cos(psi)*math.cos(theta)*math.cos(phi)-math.sin(psi)*math.sin(phi), -math.cos(psi)*math.cos(theta)*math.sin(phi)-math.sin(psi)*math.cos(phi), math.cos(psi)*math.sin(theta) ],
        #              [math.sin(psi)*math.cos(theta)*math.cos(phi)+math.cos(psi)*math.sin(phi), -math.sin(psi)*math.cos(theta)*math.sin(phi)+math.cos(psi)*math.cos(phi), math.sin(psi)*math.sin(theta) ],
        #              [-math.sin(theta)*math.cos(phi), math.sin(theta)*math.sin(phi), math.cos(theta)]])

        return R
", "src/lib/numpy/numpy_ndarray.py": "class ndarray:
  def __init__(self, shape, buffer):
    self.shape = shape;
    self.ndim = len(shape);
    self._data = buffer;
    
  def __add__(self, other):
      from numpy_internal_math import add;
      
      return add(self, other);
      
  def __str__(self):
      from numpy_internal_math import array_str;
      
      return array_str(self);
      

def _inferShapeFromList(l):
  if type(l) is list:
    if len(l) == 0:
      return (0,)
    
    shape = _inferShapeFromList(l[0])
    
    for i in range(1, len(l)):
      if shape != _inferShapeFromList(l[i]):
        raise ValueError()
    
    return (len(l),) + shape
  else:
    return tuple()

def array(obj):
  return ndarray(_inferShapeFromList(obj), obj)
  
", "src/lib/numpy/__init__.js": "var $builtinmodule = function(name) {
	var math = Sk.interop['mathjs']();
	var mod = {};
	var printPrecision=5;

	function toNativeArray(value) {
		if(Object.prototype.toString.call(value.v) === '[object Array]') {
			var result = [];
			var idx;

			for(idx = 0; idx < value.v.length; ++idx) {
				result[idx] = toNativeArray(value.v[idx]);
			}

			return result;
		} else {
			return value.v;
		}
	};

	// transform tuple/slices to index arrays
	function toNativeIndex(idx) {
		if(idx.tp$name === 'tuple') {
			var result = [];
			var i, submatrix = false, indices = [];

			for(i = 0; i < idx.v.length; ++i) {
				var tmp = toNativeIndex(idx.v[i]);

				submatrix = submatrix || tmp.submatrix;
				indices = indices.concat(tmp.indices);
			}

			return { 'submatrix': submatrix, 'indices': indices };
		} else {
			if(idx.tp$name === 'number') {
				return { submatrix: false, indices: [idx.v]};
			}

			if(idx.tp$name === 'slice') {
				return { submatrix: true, indices: [new math.type.Range(idx.start.v, idx.stop.v, idx.step.v !== null ? idx.step.v : 1)] };
			}
		}
	}

	// convert a linear index to a mathjs index array
	function linearToNativeIndex(size, idx) {
		var i, remainder = idx, total = 1, indices = [];

		for(i = size.length - 1; i >= 0 ; --i) {
			indices[i] = remainder % size[i];
			remainder = Math.floor(remainder / size[i]);
			total *= size[i];
		}

		return { 'indices': indices, 'total': total };
	}

	// translate negative indices/add missing dimensions
	function normalizeNativeIndex(size, idx) {
		if(size.length < idx.indices.length) {
			throw new Sk.builtin.IndexError('invalid index (number of indices is larger than ndarray.ndims)');
		}

		var i;
		for(i = 0; i < idx.indices.length; ++i) {
			if(math.type.Range.isRange(idx.indices[i])) {
				// clamp range
				if(idx.indices[i].end > size[i]) {
					idx.indices[i].end = size[i];
				}
			} else {
				// translate negative indices
				if(idx.indices[i] < 0) {
					idx.indices[i] = size[i] + idx.indices[i];
				}
			}
		}

		// add missing dimensions
		if(size.length > idx.indices.length) {
			var i;
			for(i = idx.indices.length; i < size.length; ++i) {
				if(size[i] > 1) {
					idx.indices[i] = new math.type.Range(0, size[i]);
					idx.submatrix = true
				} else {
					idx.indices[i] = 0;
				}
			}
		}

		return idx;
	}

	/**
	ndarray class
	TODO: can we make it iterable?
	*/
	var ndarray = function($gbl, $loc) {
		$loc.__init__ = new Sk.builtin.func(function(self, shape, data) {
			Sk.builtin.pyCheckArgs('numpy.ndarray.__init__', arguments, 2, 3);

			if(shape !== undefined) {

				try {
					self.v = math.matrix(toNativeArray(data));
				} catch(e) {
					throw new Sk.builtin.Exception(e.message);
				}

				// TODO: check shape
			} else {
				// TODO: better implementation of wrapping mathjs object in python ndarray
				self.v = data;
			}
		});

		$loc.__getitem__ = new Sk.builtin.func(function(self, key) {
			var idx = normalizeNativeIndex(self.v.size(), toNativeIndex(key));

			try {
				if(idx.submatrix) {
					return Sk.misceval.callsim(mod.ndarray, undefined, self.v.subset(math.type.Index.create(idx.indices)));
				} else {
					return Sk.builtin.nmber(self.v.get(idx.indices), Sk.builtin.nmber.float$);
				}
			} catch(e) {
				throw new Sk.builtin.Exception(e.message);
			}
		});

		$loc.__setitem__ = new Sk.builtin.func(function(self, key, value) {
			var idx = normalizeNativeIndex(self.v.size(), toNativeIndex(key));

			try {
				if(idx.submatrix) {
					self.v.subset(math.type.Index.create(idx.indices), value.v);
				} else {
					self.v.set(idx.indices, value.v);
				}
			} catch(e) {
				throw new Sk.builtin.Exception(e.message);
			}
		});

		$loc.__add__ = new Sk.builtin.func(function(self, other) {
			return Sk.misceval.callsim(mod.add, self, other);
		});

		$loc.__iadd__ = new Sk.builtin.func(function(self, other) {
			self.v = Sk.misceval.callsim(mod.add, self, other).v;
			return self
		});

		$loc.__sub__ = new Sk.builtin.func(function(self, other) {
			return Sk.misceval.callsim(mod.subtract, self, other);
		});

		$loc.__isub__ = new Sk.builtin.func(function(self, other) {
			self.v = Sk.misceval.callsim(mod.subtract, self, other).v;
			return self
		});

		$loc.__mul__ = new Sk.builtin.func(function(self, other) {
			return Sk.misceval.callsim(mod.multiply, self, other);
		});

		$loc.__imul__ = new Sk.builtin.func(function(self, other) {
			self.v = Sk.misceval.callsim(mod.multiply, self, other).v;
			return self
		});

		$loc.__rmul__ = $loc.__mul__;

		$loc.__div__ = new Sk.builtin.func(function(self, other) {
			return Sk.misceval.callsim(mod.divide, self, other);
		});

		$loc.__idiv__ = new Sk.builtin.func(function(self, other) {
			self.v = Sk.misceval.callsim(mod.divide, self, other).v;
			return self
		});

		/*
		TODO: Skulpt doesn't call __pos__
		$loc.__pos__ = new Sk.builtin.func(function(self) {
			return self;
		});
		*/

		$loc.nb$positive = function() {
			return this;
		};

		/*
		* TODO: Skulpt doesn't call __neg__
		$loc.__neg__ = new Sk.builtin.func(function(self) {
			return Sk.misceval.callsim(mod.ndarray, undefined, math.unary(self.v));
		});
		*/

		$loc.nb$negative = function() {
			return Sk.misceval.callsim(mod.ndarray, undefined, math.unary(this.v));
		};

		$loc.transpose = new Sk.builtin.func(function(self) {
			return Sk.misceval.callsim(mod.transpose, self);
		});

		$loc.inv = new Sk.builtin.func(function(self) {
			return Sk.misceval.callsim(mod.inv, self);
		});

		$loc.item = new Sk.builtin.func(function(self, key) {
			var idx = { submatrix: false, indices: [] };

			if(key === undefined) {
				var tmp = linearToNativeIndex(self.v.size(), 0)

				if(tmp.total !== 1) {
					throw new Sk.builtin.ValueError('can only convert an array  of size 1 to a Python scalar');
				}

				idx.indices = tmp.indices;
			} else if(key.tp$name === 'number' && key.skType == 'int') {
				var tmp = linearToNativeIndex(self.v.size(), key.v)

				if(key.v < 0 || key.v >= tmp.total) {
					throw new Sk.builtin.ValueError('index out of bounds');
				}

				idx.indices = tmp.indices;
			} else if(key.tp$name === 'tuple') {
				idx = normalizeNativeIndex(self.v.size(), toNativeIndex(key));
			} else {
				throw new Sk.builtin.Exception('invalid index argument of type \"' + key.tp$name + '\" for ndarray.item()!');
			}

			if(idx.submatrix) {
				throw new Sk.builtin.Exception('ndarray.item() can only be used to access scalar values!');
			}

			try {
				return Sk.builtin.nmber(self.v.get(idx.indices), Sk.builtin.nmber.float$);
			} catch(e) {
				throw new Sk.builtin.Exception(e.message);
			}
		});


		$loc.__getattr__ = new Sk.builtin.func(function(self, attr) {
			if(attr == 'ndim') return Sk.builtin.nmber(self.v.size().length, Sk.builtin.nmber.int$)
			if(attr == 'shape') return Sk.builtin.tuple(self.v.size())
			if(attr == 'T') return Sk.misceval.callsim(mod.transpose, self);
			return self.tp$getattr(attr);
		});

		$loc.__str__ = new Sk.builtin.func(function(self) {
			return Sk.misceval.callsim(mod.array_str, self);
		});
	};
	mod.ndarray = Sk.misceval.buildClass(mod, ndarray, 'ndarray', []);

	/**
	 * linalg package
	 */
	// TODO: this is still not the right way
	mod.linalg = new Sk.builtin.module();
	mod.linalg['$d'] = {
		__name__:  Sk.builtin.str('numpy.linalg'),
		inv: new Sk.builtin.func(function(array1) {
			Sk.builtin.pyCheckArgs('inv', arguments, 1);

			var result;
			try {
				result = math.inv(array1.v);
			} catch(e) {
				throw new Sk.builtin.Exception(e.message);
			}
			return Sk.misceval.callsim(mod.ndarray, undefined, result);
		}),
	};
	
	/**
	 * random package
	 */
	 
	var normal_dist = math.distribution('normal') 
	mod.random = new Sk.builtin.module();
	mod.random['$d'] = {
		__name__:  Sk.builtin.str('numpy.linalg'),
		random: new Sk.builtin.func(function(shape) {
			Sk.builtin.pyCheckArgs('random', arguments, 1, 1);
			var s = toValidShape(shape);

			var result;
			try {
				var mat=math.zeros(s.rows, s.cols);
				result = mat.map(function (value, index, v) {
					return math.random(0, 1);
				});
			} catch(e) {
				throw new Sk.builtin.Exception(e.message);
			}
			return Sk.misceval.callsim(mod.ndarray, undefined, result);
		}),
		normal: new Sk.builtin.func(function(mu, sigma, shape) {
			Sk.builtin.pyCheckArgs('normal', arguments, 0, 3);
			var mean = mu !== undefined ? mu.v : 0.0;
			var std = sigma !== undefined ? sigma.v : 1.0;
			std *= 6.0; // mathjs uses sigma 1/6
			mean -= std * 0.5; // mathjs uses mean 0.5
			var s = toValidShape(shape);
		
			var result;
			try {
				result = math.add(math.emultiply(normal_dist.random([s.rows, s.cols]), std), mean)
			} catch(e) {
				throw new Sk.builtin.Exception(e.message);
			}
			return Sk.misceval.callsim(mod.ndarray, undefined, result);
		})
	};

	/**
	 * creation functions
	 */
	mod.array = new Sk.builtin.func(function(data) {
		Sk.builtin.pyCheckArgs('array', arguments, 1);

		return Sk.misceval.callsim(mod.ndarray, Sk.builtin.tuple(), data);
	});
	
	function toValidShape(size) {
		var rows = 1, cols = 1;
		
		if(size.tp$name === 'number') {
			rows = size.v
		} else {
			if(size.v.length > 0) {
				rows = size.v[0].v
			}
			if(size.v.length > 1) {
				cols = size.v[1].v
			}
		}
		return { 'rows': rows, 'cols': cols };
	}

	mod.zeros = new Sk.builtin.func(function(shape) {
		Sk.builtin.pyCheckArgs('zeros', arguments, 1);
		var s = toValidShape(shape);
		
		var result = math.zeros(s.rows, s.cols);
		return Sk.misceval.callsim(mod.ndarray, undefined, result);
	});

	mod.ones = new Sk.builtin.func(function(shape) {
		Sk.builtin.pyCheckArgs('ones', arguments, 1);
		var s = toValidShape(shape);
		var result = math.ones(s.rows, s.cols);
		return Sk.misceval.callsim(mod.ndarray, undefined, result);
	});

	mod.identity = new Sk.builtin.func(function(n) {
		Sk.builtin.pyCheckArgs('identity', arguments, 1);

		var result = math.eye(n.v);
		return Sk.misceval.callsim(mod.ndarray, undefined, result);
	});

	mod.arange = new Sk.builtin.func(function(start,end,step) {
		Sk.builtin.pyCheckArgs('arange', arguments, 3);

		var result;
		try {
			result = math.range(start.v,end.v,step.v);
		} catch(e) {
			throw new Sk.builtin.Exception(e.message);
		}
		return Sk.misceval.callsim(mod.ndarray, undefined, result);
	});

	mod.copy = new Sk.builtin.func(function(other) {
		Sk.builtin.pyCheckArgs('copy', arguments, 1, 1);
		var result;
		try {
			result=math.clone(other.v);
		} catch(e) {
			throw new Sk.builtin.Exception(e.message);
		}
		return Sk.misceval.callsim(mod.ndarray, undefined, result);
	});

	/**
	 * arithmetic functions
	 */
	mod.add = new Sk.builtin.func(function(array1, array2) {
		Sk.builtin.pyCheckArgs('add', arguments, 2);

		var result;
		try {
			result = math.add(array1.v, array2.v);
		} catch(e) {
			throw new Sk.builtin.Exception(e.message);
		}
		return Sk.misceval.callsim(mod.ndarray, undefined, result);
	});

	mod.subtract = new Sk.builtin.func(function(array1, array2) {
		Sk.builtin.pyCheckArgs('subtract', arguments, 2);

		var result;
		try {
			result = math.subtract(array1.v, array2.v);
		} catch(e) {
			throw new Sk.builtin.Exception(e.message);
		}
		return Sk.misceval.callsim(mod.ndarray, undefined, result);
	});

	mod.multiply = new Sk.builtin.func(function(array1, array2) {
		Sk.builtin.pyCheckArgs('multiply', arguments, 2);

		var result;
		try {
			result = math.emultiply(array1.v, array2.v);
		} catch(e) {
			throw new Sk.builtin.Exception(e.message);
		}
		return Sk.misceval.callsim(mod.ndarray, undefined, result);
	});

	mod.divide = new Sk.builtin.func(function(array1, array2) {
		Sk.builtin.pyCheckArgs('divide', arguments, 2);

		var result;
		try {
			result = math.edivide(array1.v, array2.v);
		} catch(e) {
			throw new Sk.builtin.Exception(e.message);
		}
		return Sk.misceval.callsim(mod.ndarray, undefined, result);
	});

	mod.dot = new Sk.builtin.func(function(array1, array2) {
		Sk.builtin.pyCheckArgs('dot', arguments, 2);

		var result;
		try {
			result = math.multiply(array1.v, array2.v);
		} catch(e) {
			throw new Sk.builtin.Exception(e.message);
		}
		return Sk.misceval.callsim(mod.ndarray, undefined, result);
	});

	mod.cross = Sk.nativejs.func(function(x, y, axisa, axisb, axisc, axis) {
		Sk.builtin.pyCheckArgs('cross', arguments, 2);

		if(axisa) throw new Sk.builtin.Exception(\"argument axisa is not supported\");
		if(axisb) throw new Sk.builtin.Exception(\"argument axisb is not supported\");
		if(axisc) throw new Sk.builtin.Exception(\"argument axisc is not supported\");

		axis = axis ? axis.v : 1;

		var size_x = x.v.size()
		var size_y = y.v.size()

		if(size_x[axis] != 3 || size_y[axis] != 3) {
			throw new Sk.builtin.Exception(\"incompatible dimensions for cross product (dimension must be 3)\");
		}

		var result;
		try {
			if(axis == 1) {//expect row vectors
				result=math.zeros(1,3);
				result.subset(math.index(0,0), x.v._data[0][1]*y.v._data[0][2]-x.v._data[0][2]*y.v._data[0][1]);
				result.subset(math.index(0,1), x.v._data[0][2]*y.v._data[0][0]-x.v._data[0][0]*y.v._data[0][2]);
				result.subset(math.index(0,2), x.v._data[0][0]*y.v._data[0][1]-x.v._data[0][1]*y.v._data[0][0]);
			} else { //col vectors
				result=math.zeros(3,1);
				result.subset(math.index(0,0), x.v._data[1][0]*y.v._data[2][0]-x.v._data[2][0]*y.v._data[1][0]);
				result.subset(math.index(1,0), x.v._data[2][0]*y.v._data[0][0]-x.v._data[0][0]*y.v._data[2][0]);
				result.subset(math.index(2,0), x.v._data[0][0]*y.v._data[1][0]-x.v._data[1][0]*y.v._data[0][0]);
			}
		} catch(e) {
			throw new Sk.builtin.Exception(e.message);
		}
		return Sk.misceval.callsim(mod.ndarray, undefined, result);
	});

	mod.transpose = new Sk.builtin.func(function(array1) {
		Sk.builtin.pyCheckArgs('transpose', arguments, 1);

		var result;
		try {
			result = math.transpose(array1.v);
		} catch(e) {
			throw new Sk.builtin.Exception(e.message);
		}
		return Sk.misceval.callsim(mod.ndarray, undefined, result);
	});

	mod.concatenate = Sk.nativejs.func(function(array_tuple, axis) {
		Sk.builtin.pyCheckArgs('concatenate', arguments, 1, 2);
		
		axis = axis ? axis.v : 0;

		var args = [], idx, value;

		for(idx = 0; idx < array_tuple.v.length; ++idx) {
			value = array_tuple.v[idx];

			if(value.tp$name === 'number') {
				value = math.matrix([[value.v]])
			} else {
				value = value.v;
			}
			args.push(value);
		}

		// dimension argument
		args.push(axis);

		var result;
		try {
			result = math.concat.apply(undefined, args);
		} catch(e) {
			throw new Sk.builtin.Exception(e.message);
		}

		return Sk.misceval.callsim(mod.ndarray, undefined, result);
	});

	mod.hstack = new Sk.builtin.func(function(array_tuple) {
		Sk.builtin.pyCheckArgs('hstack', arguments, 1, 1);
		
		return Sk.misceval.callsim(mod.concatenate, array_tuple, Sk.builtin.nmber(1, Sk.builtin.nmber.int$));
	});

	mod.vstack = new Sk.builtin.func(function(array_tuple) {
		Sk.builtin.pyCheckArgs('vstack', arguments, 1, 1);
		
		return Sk.misceval.callsim(mod.concatenate, array_tuple, Sk.builtin.nmber(0, Sk.builtin.nmber.int$));
	});

	/**
	 * statistic functions
	 */
	mod.mean = new Sk.builtin.func(function(array1) {
		Sk.builtin.pyCheckArgs('mean', arguments, 1);

		var result;
		try {
			result = math.mean(array1.v);
		} catch(e) {
			throw new Sk.builtin.Exception(e.message);
		}
		return Sk.misceval.callsim(mod.ndarray, undefined, result);
	});

	mod.var_$rw$ = new Sk.builtin.func(function(array1) {
		Sk.builtin.pyCheckArgs('var', arguments, 1);

		var result;
		try {
			result = math['var'](array1.v);
		} catch(e) {
			throw new Sk.builtin.Exception(e.message);
		}
		return Sk.misceval.callsim(mod.ndarray, undefined, result);
	});

	mod.std = new Sk.builtin.func(function(array1) {
		Sk.builtin.pyCheckArgs('std', arguments, 1);

		var result;
		try {
			result = math.std(array1.v);
		} catch(e) {
			throw new Sk.builtin.Exception(e.message);
		}
		return Sk.misceval.callsim(mod.ndarray, undefined, result);
	});
	
	mod.cov = new Sk.builtin.func(function(array1, array2) {
		Sk.builtin.pyCheckArgs('cov', arguments, 2);
		
		if(array1.v.length != array2.v.length) {
			throw new Sk.builtin.Exception(\"time series must be equally long for covariance calculation\");
		}

		var product, mean1, mean2, result;
		try {
			product = math.emultiply(array1.v, array2.v);
			mean1 = math.mean(array1.v);
			mean2 = math.mean(array2.v);
			result = (1/(array1.v._data.length-1))*(math.sum(product)-array1.v._data.length*mean1*mean2);
		} catch(e) {
			throw new Sk.builtin.Exception(e.message);
		}
		return Sk.misceval.callsim(mod.ndarray, undefined, result);
	});

	/**
	 * logic functions
	 */
	mod.allclose = new Sk.builtin.func(function(a, b) {
		Sk.builtin.pyCheckArgs('allclose', arguments, 2, 4);

		var rtol = 1e-5, atol = 1e-8;

		if(arguments.length > 2) {
			Sk.builtin.pyCheckType('rtol', 'number', Sk.builtin.checkNumber(arguments[2]));
			rtol = arguments[2].v
		}
		if(arguments.length > 3) {
			Sk.builtin.pyCheckType('atol', 'number', Sk.builtin.checkNumber(arguments[3]));
			atol = arguments[3].v
		}

		var result = true;

		math.collection.deepMap2(a.v, b.v, function(v1, v2) {
			result = result && (Math.abs(v1 - v2) <= (atol + rtol * Math.abs(v2)));
		});

		return Sk.builtin.bool(result);
	});

	/**
	 * output functions
	 */
	 
	mod.set_printoptions = new Sk.builtin.func(function(precision,suppress) {
		Sk.builtin.pyCheckArgs('set_printoptions', arguments, 0);
		precision = typeof precision !== 'undefined' ? precision : 4;
   		suppress = typeof suppress !== 'undefined' ? suppress : true;
   		printPrecision = precision;
   		//TODO: implement suppress small numbers
   		return undefined;
	});

	mod.array_str = new Sk.builtin.func(function(array) {
		Sk.builtin.pyCheckArgs('array_str', arguments, 1);
		var str =math.format(array.v,printPrecision).replace(/\\], \\[/g, ']\
 [');
		return Sk.builtin.str(str.replace(/,/g, ''));
		//return Sk.builtin.str(math.format(array.v));
	});

	return mod;
}
", "src/builtin/this.py": "s = \"\"\"Gur Mra bs Clguba, ol Gvz Crgref

Ornhgvshy vf orggre guna htyl.
Rkcyvpvg vf orggre guna vzcyvpvg.
Fvzcyr vf orggre guna pbzcyrk.
Pbzcyrk vf orggre guna pbzcyvpngrq.
Syng vf orggre guna arfgrq.
Fcnefr vf orggre guna qrafr.
Ernqnovyvgl pbhagf.
Fcrpvny pnfrf nera'g fcrpvny rabhtu gb oernx gur ehyrf.
Nygubhtu cenpgvpnyvgl orngf chevgl.
Reebef fubhyq arire cnff fvyragyl.
Hayrff rkcyvpvgyl fvyraprq.
Va gur snpr bs nzovthvgl, ershfr gur grzcgngvba gb thrff.
Gurer fubhyq or bar-- naq cersrenoyl bayl bar --boivbhf jnl gb qb vg.
Nygubhtu gung jnl znl abg or boivbhf ng svefg hayrff lbh'er Qhgpu.
Abj vf orggre guna arire.
Nygubhtu arire vf bsgra orggre guna *evtug* abj.
Vs gur vzcyrzragngvba vf uneq gb rkcynva, vg'f n onq vqrn.
Vs gur vzcyrzragngvba vf rnfl gb rkcynva, vg znl or n tbbq vqrn.
Anzrfcnprf ner bar ubaxvat terng vqrn -- yrg'f qb zber bs gubfr!\"\"\"

d = {}
for c in (65, 97):
    for i in range(26):
        d[chr(i+c)] = chr((i+13) % 26 + c)

print \"\".join([d.get(c, c) for c in s])
", "src/lib/simulator/controller.py": "import math
import numpy as np

#when more order types are needed, a 'Order' superclass should be used

class RelativeOrder(object):
    def __init__(self, dx, dy, dz, dyaw):
        #relative movements
        self.dx = dx
        self.dy = dy
        self.dz = dz

        self.dyaw = (dyaw / 180.0) * math.pi

class MissionPlanner:
    def __init__(self):
        self.commands = []

    def forward(self, distance):
        return self._add_relative_command(distance, 0, 0, 0)

    def backward(self, distance):
        return self._add_relative_command(-distance, 0, 0, 0)

    def left(self, distance):
        return self._add_relative_command(0, distance, 0, 0)

    def right(self, distance):
        return self._add_relative_command(0, -distance, 0, 0)

    def up(self, distance):
        return self._add_relative_command(0, 0, distance, 0)

    def down(self, distance):
        return self._add_relative_command(0, 0, -distance, 0)

    def turn_left(self, angle):
        return self._add_relative_command(0, 0, 0, angle)

    def turn_right(self, angle):
        return self._add_relative_command(0, 0, 0, -angle)

    def _add_relative_command(self, dx, dy, dz, dyaw):
        self.commands.append(RelativeOrder(dx, dy, dz, dyaw))
        return self

    def add_commands(self, commands):
        #perform typecheck of appended commands
        for command in commands:
            if not isinstance(command, RelativeOrder):
                raise Exception(\"you can only add relative movement orders to the mission.\")
        self.commands += commands


class PositionController:
    command_queue     = []
    command_queue_idx = -1

    setpoint_position = np.array([[0], [0], [0]])
    setpoint_yaw      = 0.0
    done              = False

    Kp_pos = 1.0
    Kd_pos = 1.0

    Kp_yaw = 2.0
    Kd_yaw = 4.0

    Limit_xy = 2.0
    Limit_z  = 0.5

    def __init__(self, drone, commands, do_log):
        self.drone         = drone
        self.command_queue = commands
        self.do_log        = do_log

    def distance_to_setpoint(self):
        pos_diff = self.setpoint_position - self.drone.x
        yaw_diff = 5 * (self.setpoint_yaw - self.drone.theta.item(2))

        return math.sqrt(np.dot(pos_diff.transpose(), pos_diff).item(0) + yaw_diff * yaw_diff)

    def update_setpoint(self, delta):
        world_delta            = np.dot(self.drone.yaw_rotation(), np.array([[delta.dx], [delta.dy], [delta.dz]]))
        self.setpoint_position = self.setpoint_position + world_delta
        self.setpoint_yaw     += delta.dyaw

    def clamp(self, value, limit):
        return max(min(value, limit), -limit)

    def compute_input(self):
        if self.done:
            return [0, 0, 0], 0;

        if self.command_queue_idx < 0 or self.distance_to_setpoint() < 0.05:
            self.command_queue_idx += 1

            if self.command_queue_idx < len(self.command_queue):
                self.update_setpoint(self.command_queue[self.command_queue_idx])
                if self.do_log:
                    print \"updating setpoint, position:\", self.setpoint_position.transpose(), \"yaw:\", self.setpoint_yaw
            else:
                self.done = True
                if self.do_log:
                    print \"done\"

        lin_vel_cmd = self.Kp_pos * (self.setpoint_position - self.drone.x) - self.Kd_pos * self.drone.xdot;
        yaw_vel_cmd = self.Kp_yaw * (self.setpoint_yaw - self.drone.theta.item(2)) - self.Kd_yaw * self.drone.thetadot.item(2)

        return [
            self.clamp(lin_vel_cmd.item(0), self.Limit_xy),
            self.clamp(lin_vel_cmd.item(1), self.Limit_xy),
            self.clamp(lin_vel_cmd.item(2), self.Limit_z)
        ], yaw_vel_cmd


class Controller():
    '''
    Implements a PIDController
    '''

    Kp_xy = 1
    Kd_xy = 1
    Ki_xy = 0

    Kp_roll = 3
    Kd_roll = 9
    Ki_roll = 0

    Kp_pitch = 3
    Kd_pitch = 9
    Ki_pitch = 0

    Kp_yaw = 1
    Kd_yaw = 1
    Ki_yaw = 0

    Kp_z = 1
    Kd_z = 1
    Ki_z = 0

    agressiveness_xy = 0.3
    agressiveness_z  = 0.3

    dt = 0.005

    def __init__(self, drone):
        self.drone         = drone
        self.errorIntegral = np.array([[0], [0], [0]])

        # TODO: tune gains
        self.Kp_angular_rate = np.array([[3.0], [3.0], [1.0]])
        self.Kp_attitude     = np.array([[5.0], [5.0], [1.0]])
        self.Kd_attitude     = np.array([[0.0], [0.0], [0.0]])
        self.Kp_zvelocity    = 5.0

        self.Kp_lin_vel = np.array([[5.0], [5.0], [5.0]])
        self.Kd_lin_vel = np.array([[2.5], [2.5], [0]])

        self.Kp_ang_vel = 10.0
        self.Kd_ang_vel = 5.0

        self.Kp_yaw_vel = 1.0


    def calculate_control_command3(self,dt,xdot_desired, yawdot_desired):

        world_acc_cmd    = self.Kp_lin_vel * (xdot_desired - self.drone.xdot) - self.Kd_lin_vel * self.drone.xdoubledot;
        world_acc_cmd[2] = world_acc_cmd.item(2) + self.drone.g# / (math.cos(self.drone.theta.item(1))*math.cos(self.drone.theta.item(0)))
        #print \"world\", world_acc_cmd.transpose()
        body_acc_cmd     = np.dot(self.drone.rotation().transpose(), world_acc_cmd)
        body_angular_vel = self.drone.omega#np.dot(self.drone.angle_rotation_to_body(), self.drone.thetadot)
        #print \"body\", body_angular_vel.transpose()

        rates = np.array([
            [self.Kp_ang_vel * (-body_acc_cmd.item(1) / self.drone.g) - self.Kd_ang_vel * body_angular_vel.item(0)],
            [self.Kp_ang_vel * (body_acc_cmd.item(0) / self.drone.g) - self.Kd_ang_vel * body_angular_vel.item(1)],
            [self.Kp_yaw_vel * (yawdot_desired - self.drone.thetadot.item(2))]
        ]);

        T_des = world_acc_cmd.item(2) / (math.cos(self.drone.theta.item(1)) * math.cos(self.drone.theta.item(0)))
        rates = np.vstack((rates, T_des))
        ctrl  = np.dot(self.drone.AinvKinvI, rates)
        return ctrl, world_acc_cmd

    def calculate_control_command(self,dt,theta_desired,thetadot_desired,x_desired,xdot_desired):
        # TODO: implement z velocity controller feeding to desired thrust

        T_des = self.drone.g / (math.cos(self.drone.theta.item(1))*math.cos(self.drone.theta.item(0)));
        T_des += self.Kp_zvelocity * (xdot_desired.item(2) - self.drone.xdot.item(2))
        #print \"T_des\",T_des
        # attitude controller

        # angular loop
        #thetadot_desired = self.Kp_attitude * (np.dot(self.drone.yaw_rotation(), theta_desired) - self.drone.theta)# - self.Kd_attitude * self.drone.thetadot;
        thetadot_desired = self.Kp_attitude * (theta_desired - self.drone.theta);
        #print (theta_desired - self.drone.theta_in_body()).transpose(), thetadot_desired.transpose()

        thetadot_desired[2] = theta_desired.item(2)

        #print self.drone.theta.transpose(), self.drone.theta_in_body().transpose(), thetadot_desired.transpose(), self.drone.thetadot.transpose(), self.drone.thetadot_in_body().transpose();
        #print \"err\",(theta_desired - self.drone.theta_in_body()).transpose(), theta_desired.transpose(), self.drone.theta_in_body().transpose(), self.drone.theta.transpose()
        # angular rate controller
        #rates = self.Kp_angular_rate * (thetadot_desired - self.drone.thetadot)
        rates = self.Kp_angular_rate * (np.dot(self.drone.angle_rotation_to_body(), thetadot_desired) - self.drone.thetadot_in_body())
        #print (thetadot_desired - self.drone.thetadot_in_body()).transpose(), rates.transpose()
        #print \"theta_desired\", theta_desired
        #print \"self.drone.theta_in_body()\", self.drone.theta_in_body()
        #print \"thetadot_desired\",thetadot_desired
        #print \"self.drone.thetadot\",self.drone.thetadot
        #print \"self.drone.thetadot_in_body()\",self.drone.thetadot_in_body()
        rates = np.vstack((rates, T_des))
        #rates[0] = (rates[0] if rates[0] <= 1 else 1) if rates[0] >= -1 else -1;
        #rates[1] = (rates[1] if rates[1] <= 1 else 1) if rates[1] >= -1 else -1;
        #rates[2] = (rates[2] if rates[2] <= 1 else 1) if rates[2] >= -1 else -1;
        #rates[3] = (rates[3] if rates[3] <= 12 else 12) if rates[2] >= 8 else 8;
        #print \"rates1\",type(rates.item(3)),rates.item(3)
        #print \"rates2\",rates
        #print \"self.drone.AinvKinvI\", self.drone.AinvKinvI
        ctrl = np.dot(self.drone.AinvKinvI, rates);
        #print rates.transpose(), np.dot(self.drone.KA, ctrl).transpose(), ctrl.transpose();
        #ctrl = np.min(np.hstack((ctrl, np.array([[8.0], [8.0], [8.0], [8.0]]))), 1)[:,None];
        #print \"r\", rates.transpose(), ctrl.transpose();
        #print \"ctrl\", ctrl
        return ctrl

    def calculate_control_command2(self, dt, theta_desired, thetadot_desired, x_desired, xdot_desired):
        #xy control
        e_x = x_desired.item(0) - self.drone.x.item(0) + xdot_desired.item(0) - self.drone.xdot.item(0)
        e_y = x_desired.item(1) - self.drone.x.item(1) + xdot_desired.item(1) - self.drone.xdot.item(1)

        position_cmd = self.Kp_xy*(x_desired-self.drone.x)+self.Kd_xy*(xdot_desired-self.drone.xdot)
        u_x          = position_cmd.item(0)
        u_y          = position_cmd.item(1)

        R = np.array([[math.cos(self.drone.theta.item(2)), -math.sin(self.drone.theta.item(2))],
                      [math.sin(self.drone.theta.item(2)),  math.cos(self.drone.theta.item(2))]])
        u_local = np.dot(R, np.array([[u_x], [u_y]]))

        #theta_desired[0] = u_local.item(0)*self.agressiveness_xy
        #theta_desired[1] = u_local.item(1)*self.agressiveness_xy

        #yaw control
        e_yaw             = theta_desired.item(2) - self.drone.theta.item(2) #+thetadot_desired.item(2)-self.drone.thetadot.item(2)
        u_yaw             = self.Kp_yaw * (theta_desired.item(2) - self.drone.theta.item(2)) + self.Kd_yaw * (thetadot_desired.item(2)-self.drone.thetadot.item(2))
        #theta_desired[2] = math.atan2(math.sin(u_yaw), math.cos(u_yaw));
        e_yaw_norm        = -u_yaw #-math.atan2(math.sin(u_yaw), math.cos(u_yaw));
        # TODO: handle flips from +pi to -pi

        #altitude control
        e_altitude = self.Kp_z * (x_desired.item(2) - self.drone.x.item(2)) + self.Kd_z * (xdot_desired.item(2) - self.drone.xdot.item(2))
        e_z        = x_desired.item(2)-self.drone.x.item(2)+xdot_desired.item(2)-self.drone.xdot.item(2)
        print \"z err:\", e_altitude, x_desired.item(2), self.drone.x.item(2)
        qtt        = (self.drone.m * self.drone.g) / (4 * self.drone.k_t * math.cos(self.drone.theta.item(1)) * math.cos(self.drone.theta.item(0)))
        qtt        = (1 + e_altitude * self.agressiveness_z) * qtt

        e_roll  = self.Kp_roll * (theta_desired.item(0) - self.drone.theta.item(0)) + self.Kd_roll * (thetadot_desired.item(0) - self.drone.thetadot.item(0))
        e_roll  = -e_roll
        e_pitch = self.Kp_pitch * (theta_desired.item(1) - self.drone.theta.item(1)) + self.Kd_pitch * (thetadot_desired.item(1) - self.drone.thetadot.item(1))
        e_pitch = -e_pitch

        print \"errors\", e_roll, e_pitch, e_yaw_norm

        I_xx = self.drone.I.item((0, 0))
        I_yy = self.drone.I.item((1, 1))
        I_zz = self.drone.I.item((2, 2))
        #print I_xx, I_yy, I_zz
        #roll hat irgendwie keine wirkung

        #gamma = angular_velocity_motor^2
        b = self.drone.k_b
        t = self.drone.k_t
        L = self.drone.L

        gamma1 = qtt - ((2 * b * e_roll * I_xx + e_yaw_norm * I_zz * t * L) / (4 * b * t * L))
        gamma2 = qtt + ((e_yaw_norm * I_zz) / (4 * b)) - ((e_pitch * I_yy) / (2 * t * L))
        gamma3 = qtt - ((-2 * b * e_roll * I_xx + e_yaw_norm * I_zz * t * L) / (4 * b * t * L))
        gamma4 = qtt + ((e_yaw_norm * I_zz) / (4 * b)) + ((e_pitch * I_yy) / (2 * t * L))

        #Make sure we don't get above 10 Ampere in total, the rating of the Ardrone 2.0 Battery
        #@ Hovering 5.95A
        #control_commands=[abs(gamma1),abs(gamma2),abs(gamma3),abs(gamma4)]
        #if (sum(control_commands)>7):
        #    print(sum(control_commands));
        #
        #    gamma1 = (gamma1/max(control_commands))*2.4
        #    gamma2 = (gamma2/max(control_commands))*2.4
        #    gamma3 = (gamma3/max(control_commands))*2.4
        #    gamma4 = (gamma4/max(control_commands))*2.4
        #    print(\"New controls:\")
        #    print(gamma1,gamma2,gamma3,gamma4)

        return ([gamma1, gamma2, gamma3, gamma4], e_x, e_y, e_z, e_yaw, theta_desired.item(0), theta_desired.item(1), theta_desired.item(2))
", "src/lib/re/__init__.js": "var $builtinmodule = function(name)
{
    var mod = {};

    // Constants (mostly unsupported)
    mod.I = 2;
    mod.IGNORECASE = 2;
    // mod.L = 4;
    // mod.LOCALE = 4;
    mod.M = 8;
    mod.MULTILINE = 8;
    // mod.S = 16;
    // mod.DOTALL = 16;
    // mod.U = 32;
    // mod.UNICODE = 32;
    // mod.X = 64;
    // mod.VERBOSE = 64;

    var validGroups = [\"(?:\", \"(?=\", \"(?!\"];

    var convert = function(pattern) {
        var newpattern;
        var match;
        var i;

        // Look for disallowed constructs
        match = pattern.match(/\\(\\?./g);
        if (match) {
            for (i=0; i<match.length; i++) {
                if (validGroups.indexOf(match[i]) == -1) {
                    throw new Sk.builtin.ValueError(\"Disallowed group in pattern: '\"
                                                    + match[i] + \"'\");
                };
            };
        };

        newpattern = pattern.replace('/\\\\/g', '\\\\\\\\');
        newpattern = pattern.replace(/([^\\\\]){,(?![^\\[]*\\])/g, '$1{0,');

        return newpattern;
    };

    var getFlags = function(flags) {
        var jsflags = \"g\";
        if ((flags & mod.IGNORECASE) == mod.IGNORECASE) {
            jsflags += \"i\";
        };
        if ((flags & mod.MULTILINE) == mod.MULTILINE) {
            jsflags += \"m\";
        }; 
        return jsflags;
    };

    mod.split = Sk.nativejs.func(function split(pattern, string, maxsplit, flags) {
        Sk.builtin.pyCheckArgs(\"split\", arguments, 2, 4);
        if (!Sk.builtin.checkString(pattern)) {
            throw new Sk.builtin.TypeError(\"pattern must be a string\");
        };
        if (!Sk.builtin.checkString(string)) {
            throw new Sk.builtin.TypeError(\"string must be a string\");
        };
        if (maxsplit === undefined) {
            maxsplit = 0;
        };
        if (!Sk.builtin.checkNumber(maxsplit)) {
            throw new Sk.builtin.TypeError(\"maxsplit must be a number\");
        };
        if (flags === undefined) {
            flags = 0;
        };
        if (!Sk.builtin.checkNumber(flags)) {
            throw new Sk.builtin.TypeError(\"flags must be a number\");
        };

	maxsplit = Sk.builtin.asnum$(maxsplit);
        var pat = Sk.ffi.unwrapo(pattern);
        var str = Sk.ffi.unwrapo(string);
        
        // Convert pat from Python to Javascript regex syntax
        pat = convert(pat);
        //print(\"Pat: \" + pat);
        //print(\"Str: \" + str);

        var captured = !(pat.match(/^\\(.*\\)$/) === null);
        //print(\"Captured: \", captured);

        var jsflags = getFlags(flags);
        //print(\"Flags: \", jsflags);

        var regex = new RegExp(pat, jsflags);

        var result = [];
        var match;
        var index = 0;
        var splits = 0;
        while ((match = regex.exec(str)) != null) {
            //print(\"Matched '\" + match[0] + \"' at position \" + match.index + 
            //      \"; next search at \" + regex.lastIndex);
            if (match.index === regex.lastIndex) {
                // empty match
                break;
            };
            result.push(new Sk.builtin.str(str.substring(index, match.index)));
            if (captured) {
                // Add matching pattern, too
                result.push(new Sk.builtin.str(match[0]));
            };
            index = regex.lastIndex;
            splits += 1;
            if (maxsplit && (splits >= maxsplit)) {
                break;
            };
        };
        result.push(new Sk.builtin.str(str.substring(index)));

        return new Sk.builtin.list(result);
    });

    mod.findall = Sk.nativejs.func(function findall(pattern, string, flags) {
        Sk.builtin.pyCheckArgs(\"findall\", arguments, 2, 3);
        if (!Sk.builtin.checkString(pattern)) {
            throw new Sk.builtin.TypeError(\"pattern must be a string\");
        };
        if (!Sk.builtin.checkString(string)) {
            throw new Sk.builtin.TypeError(\"string must be a string\");
        };
        if (flags === undefined) {
            flags = 0;
        };
        if (!Sk.builtin.checkNumber(flags)) {
            throw new Sk.builtin.TypeError(\"flags must be a number\");
        };

        var pat = Sk.ffi.unwrapo(pattern);
        var str = Sk.ffi.unwrapo(string);
        
        // Convert pat from Python to Javascript regex syntax
        pat = convert(pat);
        //print(\"Pat: \" + pat);
        //print(\"Str: \" + str);

        var jsflags = getFlags(flags);
        //print(\"Flags: \", jsflags);

        var regex = new RegExp(pat, jsflags);

        if (pat.match(/\\$/)) {
	    var newline_at_end = new RegExp(/\
$/);
	    if (str.match(newline_at_end)) {
	        str = str.slice(0,-1);
	    }
        }

        var result = [];
        var match;
        while ((match = regex.exec(str)) != null) {
            //print(\"Matched '\" + match[0] + \"' at position \" + match.index + 
            //      \"; next search at \" + regex.lastIndex);
            // print(\"match: \" + JSON.stringify(match));
            if (match.length < 2) {
                result.push(new Sk.builtin.str(match[0]));
            } else if (match.length == 2) {
                result.push(new Sk.builtin.str(match[1]));
            } else {
                var groups = [];
                for (var i=1; i<match.length; i++) {
                    groups.push(new Sk.builtin.str(match[i]));  
                };
                result.push(new Sk.builtin.tuple(groups));
            };
            if (match.index === regex.lastIndex) {
                regex.lastIndex += 1;
            };
        };

        return new Sk.builtin.list(result);
    });


    var matchobj = function($gbl, $loc) {
        $loc.__init__ = new Sk.builtin.func(function(self,thematch, pattern, string) {
            self.thematch = thematch;
	    self.re = pattern;
	    self.string = string;
        });

	$loc.groups = new Sk.builtin.func(function(self) {
	    return new Sk.builtin.tuple(self.thematch.v.slice(1))
	});

	$loc.group = new Sk.builtin.func(function(self,grpnum) {
	    if (grpnum === undefined) {
                grpnum = 0;
            }
            else {
                grpnum = Sk.builtin.asnum$(grpnum);
            }
	    if(grpnum >= self.thematch.v.length) {
		throw new Sk.builtin.IndexError(\"Index out of range: \" + grpnum);
		}
	    return self.thematch.v[grpnum]
	});

    }

    mod.MatchObject = Sk.misceval.buildClass(mod, matchobj, 'MatchObject', []);

    // Internal function to return a Python list of strings 
    // From a JS regular expression string
    mod._findre = function(res, string) {
	res = res.replace(/([^\\\\]){,(?![^\\[]*\\])/g, '$1{0,');
        var re = eval(res);
	var patt = new RegExp('\
$');
	if (string.v.match(patt))
	    var matches = string.v.slice(0,-1).match(re);
	else
            var matches = string.v.match(re);
        retval = new Sk.builtin.list();
        if ( matches == null ) return retval;
        for (var i = 0; i < matches.length; ++i) {
            var sitem = new Sk.builtin.str(matches[i]);
            retval.v.push(sitem);
        }
        return retval;
    }

    mod.search = new Sk.builtin.func(function(pattern, string, flags) {
	Sk.builtin.pyCheckArgs('search', arguments, 2, 3);
        if (!Sk.builtin.checkString(pattern)) {
            throw new Sk.builtin.TypeError(\"pattern must be a string\");
        };
        if (!Sk.builtin.checkString(string)) {
            throw new Sk.builtin.TypeError(\"string must be a string\");
        };
	if (flags === undefined) {
            flags = 0;
        };
        if (!Sk.builtin.checkNumber(flags)) {
            throw new Sk.builtin.TypeError(\"flags must be a number\");
        };
        var res = \"/\"+pattern.v.replace(/\\//g,\"\\\\/\")+\"/\";
        lst = mod._findre(res,string);
        if ( lst.v.length < 1 ) return Sk.builtin.none.none$;
        var mob = Sk.misceval.callsim(mod.MatchObject, lst, pattern, string);
        return mob;
    });

    mod.match = new Sk.builtin.func(function(pattern, string, flags) {
	Sk.builtin.pyCheckArgs('match', arguments, 2, 3);
        if (!Sk.builtin.checkString(pattern)) {
            throw new Sk.builtin.TypeError(\"pattern must be a string\");
        };
        if (!Sk.builtin.checkString(string)) {
            throw new Sk.builtin.TypeError(\"string must be a string\");
        };
	if (flags === undefined) {
            flags = 0;
        };
        if (!Sk.builtin.checkNumber(flags)) {
            throw new Sk.builtin.TypeError(\"flags must be a number\");
        };
        var res = \"/^\"+pattern.v.replace(/\\//g,\"\\\\/\")+\"/\";
        lst = mod._findre(res,string);
        if ( lst.v.length < 1 ) return Sk.builtin.none.none$;
        var mob = Sk.misceval.callsim(mod.MatchObject, lst, pattern, string);
        return mob;
    });

    return mod;
}
", "src/lib/simulator/navdata.py": "class Navdata(object):
    '''
    drone navdata storage.

    this represents the current state of the drone.
    '''

    def __init__(self):
        # 0 means no battery, 100 means full battery
        self.battery = 50

        # 0: Unknown, 1: Init, 2: Landed, 3: Flying, 4: Hovering, 5: Test
        # 6: Taking off, 7: Goto Fix Point, 8: Landing, 9: Looping
        # Note: 3,7 seems to discriminate type of flying (isFly = 3 | 7)
        self.state = 0

        # magnetometer

        self.magX = 0
        self.magY = 0
        self.magZ = 0

        # pressure sensor
        self.pressure = 0

        # apparently, there was a temperature sensor added as well.
        self.temp = 0

        # wind sensing...
        self.wind_speed      = 0
        self.wind_angle      = 0
        self.wind_comp_angle = 0

        # left/right tilt in degrees (rotation about the X axis)
        self.rotX = 0

        # forward/backward tilt in degrees (rotation about the Y axis)
        self.rotY = 0

        # orientation in degrees (rotation about the Z axis)
        self.rotZ = 0

        # estimated altitude (cm)
        self.altd = 0

        # linear velocity (mm/sec)
        self.vx = 0

        # linear velocity (mm/sec)
        self.vy = 0

        # linear velocity (mm/sec)
        self.vz = 0

        #linear accelerations (unit: g)
        self.ax = 0
        self.ay = 0
        self.az = 0

        #motor commands (unit 0 to 255)
        self.motor1 = 0
        self.motor2 = 0
        self.motor3 = 0
        self.motor4 = 0

        #Tags in Vision Detectoion
        self.tags_count       = 0
        self.tags_type        = []
        self.tags_xc          = []
        self.tags_yc          = []
        self.tags_width       = []
        self.tags_height      = []
        self.tags_orientation = []
        self.tags_distance    = []

        #time stamp
        self.tm = 0
", "src/lib/unittest/gui.py": "import document
from unittest import TestCase

class TestCaseGui(TestCase):
	def __init__(self):
		TestCase.__init__(self)

		self.divid = document.currentDiv()
		self.mydiv = document.getElementById(self.divid)
		res = document.getElementById(self.divid+'_unit_results')
		if res:
			self.resdiv = res
			res.innerHTML = ''
		else:
			self.resdiv = document.createElement('div')
			self.resdiv.setAttribute('id',self.divid+'_unit_results')
			self.resdiv.setAttribute('class','unittest-results')
		self.mydiv.appendChild(self.resdiv)


	def main(self):
		l = document.createElement('ul')
		self.resdiv.appendChild(l)
		self.resList = l

		for func in self.tlist:
			try:
				self.setup()
				func()
				self.tearDown()
			except:
				self.appendResult('Error')
				self.numFailed += 1
		self.showSummary()

	def appendResult(self,res,actual,expected,feedback):
		if res == 'Error':
			msg = 'Error'
		elif res:
			msg = 'Pass'
			self.numPassed += 1
		else:
			msg = 'Fail: expected %s  %s ' % (str(actual),str(expected)) + feedback
			self.numFailed += 1

		pTag = document.createElement('li')
		pTag.innerHTML = msg
		self.resList.appendChild(pTag)



	def showSummary(self):
		pct = self.numPassed / (self.numPassed+self.numFailed) * 100
		pTag = document.createElement('p')
		pTag.innerHTML = \"You passed: \" + str(pct) + \"% of the tests\"
		self.resdiv.appendChild(pTag)
		if pct < 90:
			self.resdiv.setCSS('background-color','#de8e96')
		else:
			self.resdiv.setCSS('background-color','#83d382')
", "src/lib/quadrotor/__init__.py": "", "src/lib/random/__init__.js": "
/*
  I've wrapped Makoto Matsumoto and Takuji Nishimura's code in a namespace
  so it's better encapsulated. Now you can have multiple random number generators
  and they won't stomp all over eachother's state.
  
  If you want to use this as a substitute for Math.random(), use the random()
  method like so:
  
  var m = new MersenneTwister();
  var randomNumber = m.random();
  
  You can also call the other genrand_{foo}() methods on the instance.

  If you want to use a specific seed in order to get a repeatable random
  sequence, pass an integer into the constructor:

  var m = new MersenneTwister(123);

  and that will always produce the same random sequence.

  Sean McCullough (banksean@gmail.com)
*/

/* 
   A C-program for MT19937, with initialization improved 2002/1/26.
   Coded by Takuji Nishimura and Makoto Matsumoto.
 
   Before using, initialize the state by using init_genrand(seed)  
   or init_by_array(init_key, key_length).
 
   Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,
   All rights reserved.                          
 
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:
 
     1. Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
 
     2. Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
 
     3. The names of its contributors may not be used to endorse or promote 
        products derived from this software without specific prior written 
        permission.
 
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   \"AS IS\" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
   A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
   PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 
 
   Any feedback is very welcome.
   http://www.math.sci.hiroshima-u.ac.jp/~m-mat/MT/emt.html
   email: m-mat @ math.sci.hiroshima-u.ac.jp (remove space)
*/

var MersenneTwister = function(seed) {
  if (seed == undefined) {
    seed = new Date().getTime();
  } 
  /* Period parameters */  
  this.N = 624;
  this.M = 397;
  this.MATRIX_A = 0x9908b0df;   /* constant vector a */
  this.UPPER_MASK = 0x80000000; /* most significant w-r bits */
  this.LOWER_MASK = 0x7fffffff; /* least significant r bits */
 
  this.mt = new Array(this.N); /* the array for the state vector */
  this.mti=this.N+1; /* mti==N+1 means mt[N] is not initialized */

  this.init_genrand(seed);
}  
 
/* initializes mt[N] with a seed */
MersenneTwister.prototype.init_genrand = function(s) {
  this.mt[0] = s >>> 0;
  for (this.mti=1; this.mti<this.N; this.mti++) {
      var s = this.mt[this.mti-1] ^ (this.mt[this.mti-1] >>> 30);
   this.mt[this.mti] = (((((s & 0xffff0000) >>> 16) * 1812433253) << 16) + (s & 0x0000ffff) * 1812433253)
  + this.mti;
      /* See Knuth TAOCP Vol2. 3rd Ed. P.106 for multiplier. */
      /* In the previous versions, MSBs of the seed affect   */
      /* only MSBs of the array mt[].                        */
      /* 2002/01/09 modified by Makoto Matsumoto             */
      this.mt[this.mti] >>>= 0;
      /* for >32 bit machines */
  }
}
 
/* initialize by an array with array-length */
/* init_key is the array for initializing keys */
/* key_length is its length */
/* slight change for C++, 2004/2/26 */
MersenneTwister.prototype.init_by_array = function(init_key, key_length) {
  var i, j, k;
  this.init_genrand(19650218);
  i=1; j=0;
  k = (this.N>key_length ? this.N : key_length);
  for (; k; k--) {
    var s = this.mt[i-1] ^ (this.mt[i-1] >>> 30)
    this.mt[i] = (this.mt[i] ^ (((((s & 0xffff0000) >>> 16) * 1664525) << 16) + ((s & 0x0000ffff) * 1664525)))
      + init_key[j] + j; /* non linear */
    this.mt[i] >>>= 0; /* for WORDSIZE > 32 machines */
    i++; j++;
    if (i>=this.N) { this.mt[0] = this.mt[this.N-1]; i=1; }
    if (j>=key_length) j=0;
  }
  for (k=this.N-1; k; k--) {
    var s = this.mt[i-1] ^ (this.mt[i-1] >>> 30);
    this.mt[i] = (this.mt[i] ^ (((((s & 0xffff0000) >>> 16) * 1566083941) << 16) + (s & 0x0000ffff) * 1566083941))
      - i; /* non linear */
    this.mt[i] >>>= 0; /* for WORDSIZE > 32 machines */
    i++;
    if (i>=this.N) { this.mt[0] = this.mt[this.N-1]; i=1; }
  }

  this.mt[0] = 0x80000000; /* MSB is 1; assuring non-zero initial array */ 
}
 
/* generates a random number on [0,0xffffffff]-interval */
MersenneTwister.prototype.genrand_int32 = function() {
  var y;
  var mag01 = new Array(0x0, this.MATRIX_A);
  /* mag01[x] = x * MATRIX_A  for x=0,1 */

  if (this.mti >= this.N) { /* generate N words at one time */
    var kk;

    if (this.mti == this.N+1)   /* if init_genrand() has not been called, */
      this.init_genrand(5489); /* a default initial seed is used */

    for (kk=0;kk<this.N-this.M;kk++) {
      y = (this.mt[kk]&this.UPPER_MASK)|(this.mt[kk+1]&this.LOWER_MASK);
      this.mt[kk] = this.mt[kk+this.M] ^ (y >>> 1) ^ mag01[y & 0x1];
    }
    for (;kk<this.N-1;kk++) {
      y = (this.mt[kk]&this.UPPER_MASK)|(this.mt[kk+1]&this.LOWER_MASK);
      this.mt[kk] = this.mt[kk+(this.M-this.N)] ^ (y >>> 1) ^ mag01[y & 0x1];
    }
    y = (this.mt[this.N-1]&this.UPPER_MASK)|(this.mt[0]&this.LOWER_MASK);
    this.mt[this.N-1] = this.mt[this.M-1] ^ (y >>> 1) ^ mag01[y & 0x1];

    this.mti = 0;
  }

  y = this.mt[this.mti++];

  /* Tempering */
  y ^= (y >>> 11);
  y ^= (y << 7) & 0x9d2c5680;
  y ^= (y << 15) & 0xefc60000;
  y ^= (y >>> 18);

  return y >>> 0;
}
 
/* generates a random number on [0,0x7fffffff]-interval */
MersenneTwister.prototype.genrand_int31 = function() {
  return (this.genrand_int32()>>>1);
}
 
/* generates a random number on [0,1]-real-interval */
MersenneTwister.prototype.genrand_real1 = function() {
  return this.genrand_int32()*(1.0/4294967295.0); 
  /* divided by 2^32-1 */ 
}

/* generates a random number on [0,1)-real-interval */
MersenneTwister.prototype.random = function() {
  return this.genrand_int32()*(1.0/4294967296.0); 
  /* divided by 2^32 */
}
 
/* generates a random number on (0,1)-real-interval */
MersenneTwister.prototype.genrand_real3 = function() {
  return (this.genrand_int32() + 0.5)*(1.0/4294967296.0); 
  /* divided by 2^32 */
}
 
/* generates a random number on [0,1) with 53-bit resolution*/
MersenneTwister.prototype.genrand_res53 = function() { 
  var a=this.genrand_int32()>>>5, b=this.genrand_int32()>>>6; 
  return(a*67108864.0+b)*(1.0/9007199254740992.0); 
} 

/* These real versions are due to Isaku Wada, 2002/01/09 added */



var $builtinmodule = function(name)
{

    var mod = {};

    var myGenerator = new MersenneTwister();

    mod.seed = new Sk.builtin.func(function(x) {
        Sk.builtin.pyCheckArgs(\"seed\", arguments, 0, 1);
	x = Sk.builtin.asnum$(x);

        if (arguments.length > 0)
            myGenerator = new MersenneTwister(x);
        else
            myGenerator = new MersenneTwister();

	return Sk.builtin.none.none$;
    });

    mod.random = new Sk.builtin.func(function() {
        Sk.builtin.pyCheckArgs(\"random\", arguments, 0, 0);

	return new Sk.builtin.nmber(myGenerator.genrand_res53(), Sk.builtin.nmber.float$);
    });

    var toInt = function(num) {
        return num | 0;
    };

    var randrange = function(start, stop, step) {
        // Ported from CPython 2.7
        var width, n, ret;

        if (!Sk.builtin.checkInt(start)) {
            throw new Sk.builtin.ValueError(\"non-integer first argument for randrange()\");
        };

        if (stop === undefined) {
            // Random in [0, start)
            ret = toInt(myGenerator.genrand_res53() * start);
	    return new Sk.builtin.nmber(ret, Sk.builtin.nmber.int$);
        };

        if (!Sk.builtin.checkInt(stop)) {
            throw new Sk.builtin.ValueError(\"non-integer stop for randrange()\");
        };

        if (step === undefined) {
            step = 1;
        };

        width = stop - start;

        if ((step == 1) && (width > 0)) {
            // Random in [start, stop), must use toInt on product for correct results with negative ranges
            ret = start + toInt(myGenerator.genrand_res53() * width);
	    return new Sk.builtin.nmber(ret, Sk.builtin.nmber.int$);
        };

        if (step == 1) {
            throw new Sk.builtin.ValueError(\"empty range for randrange() (\" + start + \", \" + stop + \", \" + width + \")\");
        };

        if (!Sk.builtin.checkInt(step)) {
            throw new Sk.builtin.ValueError(\"non-integer step for randrange()\");
        };

        if (step > 0) {
            n = toInt((width + step - 1) / step);
        } else if (step < 0) {
            n = toInt((width + step + 1) / step);
        } else {
            throw new Sk.builtin.ValueError(\"zero step for randrange()\");
        };

        if (n <= 0) {
            throw new Sk.builtin.ValueError(\"empty range for randrange()\");
        };

        // Random in range(start, stop, step)
        ret = start + (step * toInt(myGenerator.genrand_res53() * n));
	return new Sk.builtin.nmber(ret, Sk.builtin.nmber.int$);
    };

    mod.randint = new Sk.builtin.func(function(a, b) {
        Sk.builtin.pyCheckArgs(\"randint\", arguments, 2, 2);

	a = Sk.builtin.asnum$(a);
	b = Sk.builtin.asnum$(b);
        return randrange(a, b+1);
    });

    mod.randrange = new Sk.builtin.func(function(start, stop, step) {
        Sk.builtin.pyCheckArgs(\"randrange\", arguments, 1, 3);

	start = Sk.builtin.asnum$(start);
	stop = Sk.builtin.asnum$(stop);
	step = Sk.builtin.asnum$(step);
        return randrange(start, stop, step);
    });

    mod.choice = new Sk.builtin.func(function(seq) {
        Sk.builtin.pyCheckArgs(\"choice\", arguments, 1, 1);
        Sk.builtin.pyCheckType(\"seq\", \"sequence\", Sk.builtin.checkSequence(seq));

        if (seq.sq$length !== undefined) {
            var r = toInt(myGenerator.genrand_res53() * seq.sq$length());
            return seq.mp$subscript(r);
        } else {
            throw new Sk.builtin.TypeError(\"object has no length\");
        }
    });

    mod.shuffle = new Sk.builtin.func(function(x) {
        Sk.builtin.pyCheckArgs(\"shuffle\", arguments, 1, 1);
        Sk.builtin.pyCheckType(\"x\", \"sequence\", Sk.builtin.checkSequence(x));

        if (x.sq$length !== undefined) {
            if (x.mp$ass_subscript !== undefined) {
                for (var i = x.sq$length() - 1; i > 0; i -= 1) {
                    var r = toInt(myGenerator.genrand_res53() * (i + 1));
                    var tmp = x.mp$subscript(r);
                    x.mp$ass_subscript(r, x.mp$subscript(i));
                    x.mp$ass_subscript(i, tmp);
                };
            } else {
                throw new Sk.builtin.TypeError(\"object is immutable\");
            };
        } else {
            throw new Sk.builtin.TypeError(\"object has no length\");
        };        

	return Sk.builtin.none.none$;
    });

    return mod;
}
", "src/lib/math/__init__.js": "var $builtinmodule = function(name)
{
    var mod = {};
    mod.pi = Sk.builtin.assk$(Math.PI, Sk.builtin.nmber.float$);
    mod.e =  Sk.builtin.assk$(Math.E, Sk.builtin.nmber.float$);

//	RNL	added
    mod.fabs = new Sk.builtin.func(function(x) {
        Sk.builtin.pyCheckArgs(\"fabs\", arguments, 1, 1);
        Sk.builtin.pyCheckType(\"x\", \"number\", Sk.builtin.checkNumber(x));

	return new Sk.builtin.nmber(Math.abs(Sk.builtin.asnum$(x)), Sk.builtin.nmber.float$);
    });

    mod.asin = new Sk.builtin.func(function(rad) {
        Sk.builtin.pyCheckArgs(\"asin\", arguments, 1, 1);
        Sk.builtin.pyCheckType(\"rad\", \"number\", Sk.builtin.checkNumber(rad));

	return new Sk.builtin.nmber(Math.asin(Sk.builtin.asnum$(rad)), Sk.builtin.nmber.float$);
    });

    mod.acos = new Sk.builtin.func(function(rad) {
        Sk.builtin.pyCheckArgs(\"acos\", arguments, 1, 1);
        Sk.builtin.pyCheckType(\"rad\", \"number\", Sk.builtin.checkNumber(rad));

	return new Sk.builtin.nmber(Math.acos(Sk.builtin.asnum$(rad)), Sk.builtin.nmber.float$);
    });

    mod.atan = new Sk.builtin.func(function(rad) {
        Sk.builtin.pyCheckArgs(\"atan\", arguments, 1, 1);
        Sk.builtin.pyCheckType(\"rad\", \"number\", Sk.builtin.checkNumber(rad));

	return new Sk.builtin.nmber(Math.atan(Sk.builtin.asnum$(rad)), Sk.builtin.nmber.float$);
    });

    mod.atan2 = new Sk.builtin.func(function(y, x) {
        Sk.builtin.pyCheckArgs(\"atan2\", arguments, 2, 2);
        Sk.builtin.pyCheckType(\"y\", \"number\", Sk.builtin.checkNumber(y));
        Sk.builtin.pyCheckType(\"x\", \"number\", Sk.builtin.checkNumber(x));

	return new Sk.builtin.nmber(Math.atan2(Sk.builtin.asnum$(y), Sk.builtin.asnum$(x)), Sk.builtin.nmber.float$);
    });

    mod.sin = new Sk.builtin.func(function(rad) {
        Sk.builtin.pyCheckArgs(\"sin\", arguments, 1, 1);
        Sk.builtin.pyCheckType(\"rad\", \"number\", Sk.builtin.checkNumber(rad));

	return new Sk.builtin.nmber(Math.sin(Sk.builtin.asnum$(rad)), Sk.builtin.nmber.float$);
    });

    mod.cos = new Sk.builtin.func(function(rad) {
        Sk.builtin.pyCheckArgs(\"cos\", arguments, 1, 1);
        Sk.builtin.pyCheckType(\"rad\", \"number\", Sk.builtin.checkNumber(rad));

	return new Sk.builtin.nmber(Math.cos(Sk.builtin.asnum$(rad)), Sk.builtin.nmber.float$);
    });

    mod.tan = new Sk.builtin.func(function(rad) {
        Sk.builtin.pyCheckArgs(\"tan\", arguments, 1, 1);
        Sk.builtin.pyCheckType(\"rad\", \"number\", Sk.builtin.checkNumber(rad));

	return new Sk.builtin.nmber(Math.tan(Sk.builtin.asnum$(rad)), Sk.builtin.nmber.float$);
    });

    mod.asinh = new Sk.builtin.func(function(x) {
	Sk.builtin.pyCheckArgs(\"asinh\", arguments, 1, 1);
	Sk.builtin.pyCheckType(\"x\", \"number\", Sk.builtin.checkNumber(x));

	x = Sk.builtin.asnum$(x);

	var L = x + Math.sqrt(x*x+1);

	return new Sk.builtin.nmber(Math.log(L), Sk.builtin.nmber.float$);
    });

    mod.acosh = new Sk.builtin.func(function(x) {
	Sk.builtin.pyCheckArgs(\"acosh\", arguments, 1, 1);
	Sk.builtin.pyCheckType(\"x\", \"number\", Sk.builtin.checkNumber(x));

	x = Sk.builtin.asnum$(x);

	var L = x + Math.sqrt(x*x-1);

	return new Sk.builtin.nmber(Math.log(L), Sk.builtin.nmber.float$);
    });

    mod.atanh = new Sk.builtin.func(function(x) {
	Sk.builtin.pyCheckArgs(\"atanh\", arguments, 1, 1);
	Sk.builtin.pyCheckType(\"x\", \"number\", Sk.builtin.checkNumber(x));

	x = Sk.builtin.asnum$(x);

	var L = (1+x)/(1-x);

	return new Sk.builtin.nmber(Math.log(L)/2, Sk.builtin.nmber.float$);
    });

    mod.sinh = new Sk.builtin.func(function(x) {
	Sk.builtin.pyCheckArgs(\"sinh\", arguments, 1, 1);
	Sk.builtin.pyCheckType(\"x\", \"number\", Sk.builtin.checkNumber(x));

	x = Sk.builtin.asnum$(x);

	var e = Math.E;
	var p = Math.pow(e, x);
	var n = 1/p;
	var result = (p-n)/2;

	return new Sk.builtin.nmber(result, Sk.builtin.nmber.float$);
    });

    mod.cosh = new Sk.builtin.func(function(x) {
	Sk.builtin.pyCheckArgs(\"cosh\", arguments, 1, 1);
	Sk.builtin.pyCheckType(\"x\", \"number\", Sk.builtin.checkNumber(x));

	x = Sk.builtin.asnum$(x);

	var e = Math.E;
	var p = Math.pow(e, x);
	var n = 1/p;
	var result = (p+n)/2;

	return new Sk.builtin.nmber(result, Sk.builtin.nmber.float$);
    });

    mod.tanh = new Sk.builtin.func(function(x) {
	Sk.builtin.pyCheckArgs(\"tanh\", arguments, 1, 1);
	Sk.builtin.pyCheckType(\"x\", \"number\", Sk.builtin.checkNumber(x));

	x = Sk.builtin.asnum$(x);

	var e = Math.E;
	var p = Math.pow(e, x);
	var n = 1/p;
	var result = ((p-n)/2)/((p+n)/2);

	return new Sk.builtin.nmber(result, Sk.builtin.nmber.float$);
    });

    mod.ceil = new Sk.builtin.func(function(x) {
        Sk.builtin.pyCheckArgs(\"ceil\", arguments, 1, 1);
        Sk.builtin.pyCheckType(\"x\", \"number\", Sk.builtin.checkNumber(x));

	return new Sk.builtin.nmber(Math.ceil(Sk.builtin.asnum$(x)), Sk.builtin.nmber.float$);
    });

    mod.floor = new Sk.builtin.func(function(x) {
        Sk.builtin.pyCheckArgs(\"floor\", arguments, 1, 1);
        Sk.builtin.pyCheckType(\"x\", \"number\", Sk.builtin.checkNumber(x));

	return new Sk.builtin.nmber(Math.floor(Sk.builtin.asnum$(x)), Sk.builtin.nmber.float$);
    });

    mod.sqrt = new Sk.builtin.func(function(x) {
        Sk.builtin.pyCheckArgs(\"sqrt\", arguments, 1, 1);
        Sk.builtin.pyCheckType(\"x\", \"number\", Sk.builtin.checkNumber(x));

	return new Sk.builtin.nmber(Math.sqrt(Sk.builtin.asnum$(x)), Sk.builtin.nmber.float$);
    });

    mod.trunc = new Sk.builtin.func(function(x) {
        Sk.builtin.pyCheckArgs(\"trunc\", arguments, 1, 1);
        Sk.builtin.pyCheckType(\"x\", \"number\", Sk.builtin.checkNumber(x));

        return new Sk.builtin.nmber(Sk.builtin.asnum$(x)|0, Sk.builtin.nmber.int$);
    });

    mod.log = new Sk.builtin.func(function(x, base) {
        Sk.builtin.pyCheckArgs(\"log\", arguments, 1, 2);
        Sk.builtin.pyCheckType(\"x\", \"number\", Sk.builtin.checkNumber(x));

        if (base === undefined) {
	    return new Sk.builtin.nmber(Math.log(Sk.builtin.asnum$(x)), Sk.builtin.nmber.float$);
        } else {
            Sk.builtin.pyCheckType(\"base\", \"number\", Sk.builtin.checkNumber(base));
            var ret = Math.log(Sk.builtin.asnum$(x)) / Math.log(Sk.builtin.asnum$(base));
	    return new Sk.builtin.nmber(ret, Sk.builtin.nmber.float$);
        }
    });

    mod.log10 = new Sk.builtin.func(function(x) {
        Sk.builtin.pyCheckArgs(\"log10\", arguments, 1, 1);
        Sk.builtin.pyCheckType(\"x\", \"number\", Sk.builtin.checkNumber(x));

        var ret = Math.log(Sk.builtin.asnum$(x)) / Math.log(10);
	return new Sk.builtin.nmber(ret, Sk.builtin.nmber.float$);
    });

    mod.exp = new Sk.builtin.func(function(x) {
        Sk.builtin.pyCheckArgs(\"exp\", arguments, 1, 1);
        Sk.builtin.pyCheckType(\"x\", \"number\", Sk.builtin.checkNumber(x));

	return new Sk.builtin.nmber(Math.exp(Sk.builtin.asnum$(x)), Sk.builtin.nmber.float$);
    });

    mod.pow = new Sk.builtin.func(function(x,y) {
        Sk.builtin.pyCheckArgs(\"pow\", arguments, 2, 2);
        Sk.builtin.pyCheckType(\"x\", \"number\", Sk.builtin.checkNumber(x));
        Sk.builtin.pyCheckType(\"y\", \"number\", Sk.builtin.checkNumber(y));

	return new Sk.builtin.nmber(Math.pow(Sk.builtin.asnum$(x), Sk.builtin.asnum$(y)), Sk.builtin.nmber.float$);
    });

    mod.radians = new Sk.builtin.func(function(deg) {
        Sk.builtin.pyCheckArgs(\"radians\", arguments, 1, 1);
        Sk.builtin.pyCheckType(\"deg\", \"number\", Sk.builtin.checkNumber(deg));

	var ret = Math.PI / 180.0 * Sk.builtin.asnum$(deg);
	return new Sk.builtin.nmber(ret, Sk.builtin.nmber.float$);
    });

    mod.degrees = new Sk.builtin.func(function(rad) {
        Sk.builtin.pyCheckArgs(\"degrees\", arguments, 1, 1);
        Sk.builtin.pyCheckType(\"rad\", \"number\", Sk.builtin.checkNumber(rad));

	var ret = 180.0 / Math.PI * Sk.builtin.asnum$(rad);
	return new Sk.builtin.nmber(ret, Sk.builtin.nmber.float$);
    });

    mod.hypot = new Sk.builtin.func(function(x, y) {
	Sk.builtin.pyCheckArgs(\"hypot\", arguments, 2, 2);
        Sk.builtin.pyCheckType(\"x\", \"number\", Sk.builtin.checkNumber(x));
	Sk.builtin.pyCheckType(\"y\", \"number\", Sk.builtin.checkNumber(y));

		x = Sk.builtin.asnum$(x);
		y = Sk.builtin.asnum$(y);
	return new Sk.builtin.nmber(Math.sqrt((x*x)+(y*y)), Sk.builtin.nmber.float$);
    });

	mod.factorial = new Sk.builtin.func(function(x) {
	    Sk.builtin.pyCheckArgs(\"factorial\", arguments, 1, 1);
            Sk.builtin.pyCheckType(\"x\", \"number\", Sk.builtin.checkNumber(x));

		x = Math.floor(Sk.builtin.asnum$(x));
		var r = 1;
		for (var i = 2; i <= x; i++)
			r *= i;
		return new Sk.builtin.nmber(r, Sk.builtin.nmber.int$);
	});

    return mod;
}"}}