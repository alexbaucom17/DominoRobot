# !/usr/bin/env python
# -*- coding: utf-8 -*-
# vispy: gallery 1

"""
Demonstrating a cloud of points.
"""

import numpy as np

from vispy import gloo
from vispy import app
from vispy import visuals
from vispy.util.transforms import perspective, translate, rotate

import math
import random



vert = """
#version 120
// Uniforms
// ------------------------------------
uniform mat4 u_model;
uniform mat4 u_view;
uniform mat4 u_projection;
uniform float u_linewidth;
uniform float u_antialias;
uniform float u_size;
// Attributes
// ------------------------------------
attribute vec3  a_position;
attribute vec4  a_fg_color;
attribute vec4  a_bg_color;
attribute float a_size;
// Varyings
// ------------------------------------
varying vec4 v_fg_color;
varying vec4 v_bg_color;
varying float v_size;
varying float v_linewidth;
varying float v_antialias;
void main (void) {
    v_size = a_size * u_size;
    v_linewidth = u_linewidth;
    v_antialias = u_antialias;
    v_fg_color  = a_fg_color;
    v_bg_color  = a_bg_color;
    gl_Position = u_projection * u_view * u_model * vec4(a_position,1.0);
    gl_PointSize = v_size + 2*(v_linewidth + 1.5*v_antialias);
}
"""

frag = """
#version 120
// Constants
// ------------------------------------
// Varyings
// ------------------------------------
varying vec4 v_fg_color;
varying vec4 v_bg_color;
varying float v_size;
varying float v_linewidth;
varying float v_antialias;
// Functions
// ------------------------------------
// ----------------
float disc(vec2 P, float size)
{
    float r = length((P.xy - vec2(0.5,0.5))*size);
    r -= v_size/2;
    return r;
}
// ----------------
float arrow_right(vec2 P, float size)
{
    float r1 = abs(P.x -.50)*size + abs(P.y -.5)*size - v_size/2;
    float r2 = abs(P.x -.25)*size + abs(P.y -.5)*size - v_size/2;
    float r = max(r1,-r2);
    return r;
}
// ----------------
float ring(vec2 P, float size)
{
    float r1 = length((gl_PointCoord.xy - vec2(0.5,0.5))*size) - v_size/2;
    float r2 = length((gl_PointCoord.xy - vec2(0.5,0.5))*size) - v_size/4;
    float r = max(r1,-r2);
    return r;
}
// ----------------
float clober(vec2 P, float size)
{
    const float PI = 3.14159265358979323846264;
    const float t1 = -PI/2;
    const vec2  c1 = 0.2*vec2(cos(t1),sin(t1));
    const float t2 = t1+2*PI/3;
    const vec2  c2 = 0.2*vec2(cos(t2),sin(t2));
    const float t3 = t2+2*PI/3;
    const vec2  c3 = 0.2*vec2(cos(t3),sin(t3));
    float r1 = length((gl_PointCoord.xy- vec2(0.5,0.5) - c1)*size);
    r1 -= v_size/3;
    float r2 = length((gl_PointCoord.xy- vec2(0.5,0.5) - c2)*size);
    r2 -= v_size/3;
    float r3 = length((gl_PointCoord.xy- vec2(0.5,0.5) - c3)*size);
    r3 -= v_size/3;
    float r = min(min(r1,r2),r3);
    return r;
}
// ----------------
float square(vec2 P, float size)
{
    float r = max(abs(gl_PointCoord.x -.5)*size,
                  abs(gl_PointCoord.y -.5)*size);
    r -= v_size/2;
    return r;
}
// ----------------
float diamond(vec2 P, float size)
{
    float r = abs(gl_PointCoord.x -.5)*size + abs(gl_PointCoord.y -.5)*size;
    r -= v_size/2;
    return r;
}
// ----------------
float vbar(vec2 P, float size)
{
    float r1 = max(abs(gl_PointCoord.x -.75)*size,
                   abs(gl_PointCoord.x -.25)*size);
    float r3 = max(abs(gl_PointCoord.x -.5)*size,
                   abs(gl_PointCoord.y -.5)*size);
    float r = max(r1,r3);
    r -= v_size/2;
    return r;
}
// ----------------
float hbar(vec2 P, float size)
{
    float r2 = max(abs(gl_PointCoord.y -.75)*size,
                   abs(gl_PointCoord.y -.25)*size);
    float r3 = max(abs(gl_PointCoord.x -.5)*size,
                   abs(gl_PointCoord.y -.5)*size);
    float r = max(r2,r3);
    r -= v_size/2;
    return r;
}
// ----------------
float cross(vec2 P, float size)
{
    float r1 = max(abs(gl_PointCoord.x -.75)*size,
                   abs(gl_PointCoord.x -.25)*size);
    float r2 = max(abs(gl_PointCoord.y -.75)*size,
                   abs(gl_PointCoord.y -.25)*size);
    float r3 = max(abs(gl_PointCoord.x -.5)*size,
                   abs(gl_PointCoord.y -.5)*size);
    float r = max(min(r1,r2),r3);
    r -= v_size/2;
    return r;
}
// Main
// ------------------------------------
void main()
{
    float size = v_size +2*(v_linewidth + 1.5*v_antialias);
    float t = v_linewidth/2.0-v_antialias;
    float r = disc(gl_PointCoord, size);
    float d = abs(r) - t;
    if( r > (v_linewidth/2.0+v_antialias))
    {
        discard;
    }
    else if( d < 0.0 )
    {
       gl_FragColor = v_fg_color;
    }
    else
    {
        float alpha = d/v_antialias;
        alpha = exp(-alpha*alpha);
        if (r > 0)
            gl_FragColor = vec4(v_fg_color.rgb, alpha*v_fg_color.a);
        else
            gl_FragColor = mix(v_bg_color, v_fg_color, alpha);
    }
}
"""

vs = """
uniform mat4   u_model;         // Model matrix
uniform mat4   u_view;          // View matrix
uniform mat4   u_projection;    // Projection matrix
attribute vec3 a_position;
attribute vec4  a_fg_color;
attribute vec4  a_bg_color;
attribute float a_size;

varying vec4 v_fg_color;

void main()
{
    v_fg_color = a_fg_color;
    gl_Position = u_projection * u_view * u_model * vec4(a_position, 1.0);
    //gl_LineWidth = a_linewidth;
}
"""

fs = """
varying vec4 v_fg_color;
void main()
{
//    gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0);
    gl_FragColor = v_fg_color;
}
"""

# ------------------------------------------------------------ Canvas class ---
class Canvas(app.Canvas):

    def __init__(self):

        self.pointlimiter = 500

        app.Canvas.__init__(self, keys='interactive', size=(800, 600))
        ps = self.pixel_scale


        self.translateZ = 50
        self.view = translate((0, 0, -self.translateZ))
        self.model = np.eye(4, dtype=np.float32)
        self.projection = np.eye(4, dtype=np.float32)

        self.program_e = gloo.Program(vs, fs)
        self.program = gloo.Program(vert, frag)

        self.apply_zoom()
        u_linewidth = 1.0 * 0
        u_antialias = 1.0 * 0

        self.program['u_linewidth'] = u_linewidth
        self.program['u_antialias'] = u_antialias
        self.program['u_model'] = self.model
        self.program['u_view'] = self.view
        self.program['u_size'] = 5 / self.translateZ

        self.program_e['u_model'] = self.model
        self.program_e['u_view'] = self.view
        self.program_e['u_size'] = 5 / self.translateZ

        self.theta = 0
        self.phi = 0
        self.psi = 0

        gloo.set_state('translucent', clear_color='white')

        self.timer = app.Timer(interval='0.01', connect=self.on_timer, start=True)

        self.iteratorx = 0.0
        self.iteratory = 0.0
        self.iteratorz = 0.0

        self.prevMouseEventCoord = [0,0]

        self.translateX = 0
        self.translateY = 0

        self.edgesstatic = list()

        n=5
        self.datastatic = np.zeros(4, [('a_position', np.float32, 3),
                            ('a_bg_color', np.float32, 4),
                            ('a_fg_color', np.float32, 4),
                            ('a_size', np.float32, 1)])
        # self.datastatic['a_position'] = 0.25 * np.random.randn(n, 3)
        self.datastatic['a_fg_color'] = 0, 0, 0, 1
        self.datastatic['a_size'] = [10]*4
        # 

        self.datastatic[0]['a_position'] = np.array([0,0,0])
        self.datastatic[0]['a_bg_color'] = 0,0,0,1

        self.datastatic[1]['a_position'] = np.array([10,0,0])
        self.datastatic[1]['a_bg_color'] = 1,0,0,1

        self.datastatic[2]['a_position'] = np.array([0,10,0])
        self.datastatic[2]['a_bg_color'] = 0,1,0,1

        self.datastatic[3]['a_position'] = np.array([0,0,10])
        self.datastatic[3]['a_bg_color'] = 0,0,1,1


        self.edgesstatic = [[0,1],[0,2],[0,3]]

        limleft = -100
        limright = 100
        space = np.linspace(limleft, limright, num = int((limright-limleft)/0.1)+1)
        

        for i in space:
            # print (i)
            tmp = np.zeros(1, [('a_position', np.float32, 3),
                            ('a_bg_color', np.float32, 4),
                            ('a_fg_color', np.float32, 4),
                            ('a_size', np.float32, 1)])
            tmp['a_position'] = np.array([limleft, i, 0])
            tmp['a_fg_color'] = 0.5,0.5,0.5,1
            self.datastatic = np.append(self.datastatic, tmp)


            tmp = np.zeros(1, [('a_position', np.float32, 3),
                            ('a_bg_color', np.float32, 4),
                            ('a_fg_color', np.float32, 4),
                            ('a_size', np.float32, 1)])
            tmp['a_position'] = np.array([limright, i, 0])
            tmp['a_fg_color'] = 0.5,0.5,0.5,1
            self.datastatic = np.append(self.datastatic, tmp)

            self.edgesstatic.append([len(self.datastatic)-2, len(self.datastatic)-1])

        for i in space:
            tmp = np.zeros(1, [('a_position', np.float32, 3),
                            ('a_bg_color', np.float32, 4),
                            ('a_fg_color', np.float32, 4),
                            ('a_size', np.float32, 1)])
            tmp['a_position'] = np.array([i, limleft, 0])
            tmp['a_fg_color'] = 0.5,0.5,0.5,1
            self.datastatic = np.append(self.datastatic, tmp)


            tmp = np.zeros(1, [('a_position', np.float32, 3),
                            ('a_bg_color', np.float32, 4),
                            ('a_fg_color', np.float32, 4),
                            ('a_size', np.float32, 1)])
            tmp['a_position'] = np.array([i, limright, 0])
            tmp['a_fg_color'] = 0.5,0.5,0.5,1
            self.datastatic = np.append(self.datastatic, tmp)

            self.edgesstatic.append([len(self.datastatic)-2, len(self.datastatic)-1])

        # Create vertices
        import random
        n = 0
        self.data = np.zeros(n, [('a_position', np.float32, 3),
                            ('a_bg_color', np.float32, 4),
                            ('a_fg_color', np.float32, 4),
                            ('a_size', np.float32, 1)])
        self.data['a_position'] = 10 * np.random.randn(n, 3)
        self.data['a_bg_color'] = np.random.uniform(0.85, 1.00, (n, 4))
        self.data['a_fg_color'] = 0, 0, 0, 1
        self.data['a_size'] = [10]*n

        self.edges = self.edgesstatic

        self.data = np.append(self.datastatic, self.data)
        self.vbo = gloo.VertexBuffer(self.data)
        self.index = gloo.IndexBuffer(self.edges)
        
        self.program.bind(self.vbo)
        self.program_e.bind(self.vbo)







        self.usdata = list()
        self.imudata = list()
        self.showus = True
        self.showimu = True

        self.show()

    def on_key_press(self, event):
        if event.text == ' ':
            if self.timer.running:
                self.timer.stop()
            else:
                self.timer.start()

    def on_timer(self, event):
        self.program.bind(gloo.VertexBuffer(self.data))
        self.program_e.bind(gloo.VertexBuffer(self.data))
        self.index = gloo.IndexBuffer(self.edges)
        gloo.clear(color=True, depth=True)

        self.update()

    def on_resize(self, event):
        self.apply_zoom()

    def on_mouse_wheel(self, event):
        self.translateZ -= event.delta[1]
        self.translateZ = max(2, self.translateZ)
        self.view = translate((self.translateX, self.translateY, -self.translateZ))

        self.program['u_view'] = self.view
        self.program['u_size'] = 5 / self.translateZ

        self.program_e['u_view'] = self.view
        # self.program_e['u_size'] = 5 / self.translateZ
        
        self.update()

    def on_draw(self, event):
        gloo.clear()
        
        self.program.draw('points')
        self.program_e.draw('lines', self.index)

    def apply_zoom(self):
        gloo.set_viewport(0, 0, self.physical_size[0], self.physical_size[1])
        self.projection = perspective(45.0, self.size[0] /
                                      float(self.size[1]), 1.0, 1000.0)
        self.program['u_projection'] = self.projection
        self.program_e['u_projection'] = self.projection

    def on_mouse_move(self, event):
        dx = event.pos[0] - self.prevMouseEventCoord[0]
        dy = event.pos[1] - self.prevMouseEventCoord[1]
        if (event.button==1):
            self.translateX += dx/100.0
            self.translateY += -dy/100.0
            # print(dx,dy)
            self.view = translate((self.translateX, self.translateY, -self.translateZ))
            self.program['u_view'] = self.view
            self.program_e['u_view'] = self.view
            # self.print_mouse_event(event, 'Mouse release')
        elif (event.button==2):
            self.phi += dx/10.0
            self.psi += dy/10.0
            # print(dx,dy)
            self.model = np.dot(rotate(self.theta, (0, 0, 1)), np.dot(rotate(self.phi, (0, 1, 0)),rotate(self.psi, (1, 0, 0))))
            self.program['u_model'] = self.model
            self.program_e['u_model'] = self.model
            # self.print_mouse_event(event, 'Mouse release')
        elif (event.button==3):
            self.theta += dx/10.0
            # print(dx,dy)
            self.model = np.dot(rotate(self.theta, (0, 0, 1)), np.dot(rotate(self.phi, (0, 1, 0)),rotate(self.psi, (1, 0, 0))))
            self.program['u_model'] = self.model
            self.program_e['u_model'] = self.model
            # self.print_mouse_event(event, 'Mouse release')

        self.update()
        self.prevMouseEventCoord = event.pos

    def on_key_press(self, event):
        if(event.key=='m'):
            self.fullerase()
        if (event.key=='i'):
            self.showimu = not self.showimu
            self.showpoints()
        if (event.key=='u'):
            self.showus = not self.showus
            print ('showpos ', str (self.showimu), ' ', str (self.showus))
            self.showpoints()
        if (event.key=='1'):
            self.pointlimiter=100
        if (event.key=='2'):
            self.pointlimiter=200
        if (event.key=='3'):
            self.pointlimiter=300
        if (event.key=='4'):
            self.pointlimiter=400
        if (event.key=='5'):
            self.pointlimiter=500
        if (event.key=='6'):
            self.pointlimiter=10000
        print (self.pointlimiter)



    def print_mouse_event(self, event, what):
        """ print mouse events for debugging purposes """
        print('%s - pos: %r, button: %s,  delta: %r' %
              (what, event.pos, event.button, event.delta))


    def fullerase(self):
        self.data = self.datastatic
        self.edges = self.edgesstatic
        self.usdata = list()
        self.imudata = list()

    def showpoints(self):
        self.data = self.datastatic
        if (self.showimu):
            for i in self.imudata:
                self.addPointMod1(i)
        if (self.showus):
            for i in self.usdata:
                self.addPointUltrasoundBase(i)
            



######################


    def addPoint(self, p):
        n=1
        dataapp = np.zeros(n, [('a_position', np.float32, 3),
                            ('a_bg_color', np.float32, 4),
                            ('a_fg_color', np.float32, 4),
                            ('a_size', np.float32, 1)])
        dataapp['a_position'] = np.array(p[0:3])
        # print (dataapp['a_position'])
        # dataapp['a_bg_color'] = np.append(np.random.uniform(0.8, 0.95, (n, 3)), np.array(1))
        dataapp['a_bg_color'] = 0,0,1,1
        dataapp['a_fg_color'] = 0, 0, 0, 1
        dataapp['a_size'] = [5]*n
        self.data = np.append(self.data, dataapp)

    def addPointUltrasound(self, p):
        n=1
        dataapp = np.zeros(n, [('a_position', np.float32, 3),
                            ('a_bg_color', np.float32, 4),
                            ('a_fg_color', np.float32, 4),
                            ('a_size', np.float32, 1)])
        dataapp['a_position'] = np.array(p[0:3])
        # print (dataapp['a_position'])
        dataapp['a_bg_color'] = 0.5, 0.5, 0.5, 0.5
        dataapp['a_fg_color'] = 0, 0, 0, 1
        dataapp['a_size'] = [10]*n
        self.data = np.append(self.data, dataapp)

    def addPointUltrasound63(self, p):
        n=1
        dataapp = np.zeros(n, [('a_position', np.float32, 3),
                            ('a_bg_color', np.float32, 4),
                            ('a_fg_color', np.float32, 4),
                            ('a_size', np.float32, 1)])
        dataapp['a_position'] = np.array(p[0:3])
        # print (dataapp['a_position'])
        dataapp['a_bg_color'] = 0.5, 0.5, 0.5, 0.5
        dataapp['a_fg_color'] = 0, 0, 0, 1
        dataapp['a_size'] = [10]*n
        self.data = np.append(self.data, dataapp)


    def addPointUltrasoundBase(self, p):
        n=1
        dataapp = np.zeros(n, [('a_position', np.float32, 3),
                            ('a_bg_color', np.float32, 4),
                            ('a_fg_color', np.float32, 4),
                            ('a_size', np.float32, 1)])
        dataapp['a_position'] = np.array(p[0:3])
        # print (dataapp['a_position'])
        dataapp['a_bg_color'] = 1, 0, 0, 1
        dataapp['a_fg_color'] = 0, 0, 0, 1
        dataapp['a_size'] = [15]*n
        self.data = np.append(self.data, dataapp)


    def addPointIMU(self, p):
        n=1
        dataapp = np.zeros(n, [('a_position', np.float32, 3),
                            ('a_bg_color', np.float32, 4),
                            ('a_fg_color', np.float32, 4),
                            ('a_size', np.float32, 1)])
        dataapp['a_position'] = np.array(p[0:3])
        # print (dataapp['a_position'])
        dataapp['a_bg_color'] = 0, 1, 0, 1
        dataapp['a_fg_color'] = 0, 0, 0, 1
        dataapp['a_size'] = [7]*n
        self.data = np.append(self.data, dataapp)

    def addPointIMURetrospective(self, p):
        n=1
        dataapp = np.zeros(n, [('a_position', np.float32, 3),
                            ('a_bg_color', np.float32, 4),
                            ('a_fg_color', np.float32, 4),
                            ('a_size', np.float32, 1)])
        dataapp['a_position'] = np.array(p[0:3])
        # print (dataapp['a_position'])
        dataapp['a_bg_color'] = 0, 1, 1, 1
        dataapp['a_fg_color'] = 0, 0, 0, 1
        dataapp['a_size'] = [3]*n
        self.data = np.append(self.data, dataapp)

    def addPointMod1(self, p):
        n=1
        dataapp = np.zeros(n, [('a_position', np.float32, 3),
                            ('a_bg_color', np.float32, 4),
                            ('a_fg_color', np.float32, 4),
                            ('a_size', np.float32, 1)])
        dataapp['a_position'] = np.array(p[0:3])
        # print (dataapp['a_position'])
        # dataapp['a_bg_color'] = np.append(np.random.uniform(0.8, 0.95, (n, 3)), np.array(1))
        dataapp['a_bg_color'] = 1,0,1,1
        dataapp['a_fg_color'] = 0, 0, 0, 1
        dataapp['a_size'] = [15]*n
        self.data = np.append(self.data, dataapp)


    def addPointBlue(self, p):
        n=1
        dataapp = np.zeros(n, [('a_position', np.float32, 3),
                            ('a_bg_color', np.float32, 4),
                            ('a_fg_color', np.float32, 4),
                            ('a_size', np.float32, 1)])
        dataapp['a_position'] = np.array(p[0:3])
        # print (dataapp['a_position'])
        # dataapp['a_bg_color'] = np.append(np.random.uniform(0.8, 0.95, (n, 3)), np.array(1))
        dataapp['a_bg_color'] = 0,0,1,1
        dataapp['a_fg_color'] = 0, 0, 0, 1
        dataapp['a_size'] = [10]*n
        self.data = np.append(self.data, dataapp)

    def addPointRed(self, p):
        n=1
        dataapp = np.zeros(n, [('a_position', np.float32, 3),
                            ('a_bg_color', np.float32, 4),
                            ('a_fg_color', np.float32, 4),
                            ('a_size', np.float32, 1)])
        dataapp['a_position'] = np.array(p[0:3])
        # print (dataapp['a_position'])
        # dataapp['a_bg_color'] = np.append(np.random.uniform(0.8, 0.95, (n, 3)), np.array(1))
        dataapp['a_bg_color'] = 1,0,0,1
        dataapp['a_fg_color'] = 0, 0, 0, 1
        dataapp['a_size'] = [10]*n
        self.data = np.append(self.data, dataapp)

    def addPointGreen(self, p):
        n=1
        dataapp = np.zeros(n, [('a_position', np.float32, 3),
                            ('a_bg_color', np.float32, 4),
                            ('a_fg_color', np.float32, 4),
                            ('a_size', np.float32, 1)])
        dataapp['a_position'] = np.array(p[0:3])
        # print (dataapp['a_position'])
        # dataapp['a_bg_color'] = np.append(np.random.uniform(0.8, 0.95, (n, 3)), np.array(1))
        dataapp['a_bg_color'] = 0,1,0,1
        dataapp['a_fg_color'] = 0, 0, 0, 1
        dataapp['a_size'] = [10]*n
        self.data = np.append(self.data, dataapp)

    def addPointBlue2(self, p):
        n=1
        dataapp = np.zeros(n, [('a_position', np.float32, 3),
                            ('a_bg_color', np.float32, 4),
                            ('a_fg_color', np.float32, 4),
                            ('a_size', np.float32, 1)])
        dataapp['a_position'] = np.array(p[0:3])
        # print (dataapp['a_position'])
        # dataapp['a_bg_color'] = np.append(np.random.uniform(0.8, 0.95, (n, 3)), np.array(1))
        dataapp['a_bg_color'] = 0,0,1,1
        dataapp['a_fg_color'] = 0, 0, 0, 1
        dataapp['a_size'] = [20]*n
        self.data = np.append(self.data, dataapp)

    def addPointRed2(self, p):
        n=1
        dataapp = np.zeros(n, [('a_position', np.float32, 3),
                            ('a_bg_color', np.float32, 4),
                            ('a_fg_color', np.float32, 4),
                            ('a_size', np.float32, 1)])
        dataapp['a_position'] = np.array(p[0:3])
        # print (dataapp['a_position'])
        # dataapp['a_bg_color'] = np.append(np.random.uniform(0.8, 0.95, (n, 3)), np.array(1))
        dataapp['a_bg_color'] = 1,0,0,1
        dataapp['a_fg_color'] = 0, 0, 0, 1
        dataapp['a_size'] = [25]*n
        self.data = np.append(self.data, dataapp)

    def addPointGreen2(self, p):
        n=1
        dataapp = np.zeros(n, [('a_position', np.float32, 3),
                            ('a_bg_color', np.float32, 4),
                            ('a_fg_color', np.float32, 4),
                            ('a_size', np.float32, 1)])
        dataapp['a_position'] = np.array(p[0:3])
        # print (dataapp['a_position'])
        # dataapp['a_bg_color'] = np.append(np.random.uniform(0.8, 0.95, (n, 3)), np.array(1))
        dataapp['a_bg_color'] = 0,1,0,1
        dataapp['a_fg_color'] = 0, 0, 0, 1
        dataapp['a_size'] = [20]*n
        self.data = np.append(self.data, dataapp)

    def addPointMod2(self, p, t):
        n=1
        dataapp = np.zeros(n, [('a_position', np.float32, 3),
                            ('a_bg_color', np.float32, 4),
                            ('a_fg_color', np.float32, 4),
                            ('a_size', np.float32, 1)])
        dataapp['a_position'] = np.array(p[0:3])
        # print (dataapp['a_position'])
        # dataapp['a_bg_color'] = np.append(np.random.uniform(0.8, 0.95, (n, 3)), np.array(1))
        dataapp['a_bg_color'] = 1,0,1*t,1
        dataapp['a_fg_color'] = 0, 0, 0, 1
        dataapp['a_size'] = [2]*n
        self.data = np.append(self.data, dataapp)