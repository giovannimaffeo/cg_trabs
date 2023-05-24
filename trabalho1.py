import sys
import numpy as np
from math import *
from OpenGL.GLUT import *
from OpenGL.GL import *
from OpenGL.GLU import *
from pyrr.matrix44 import *

shapes = []

def calculate_norm(v):
    norm = sqrt(v[0] ** 2 + v[1] ** 2)
    return norm

def angle_between_vectors(v1, v2):
    v1_norm = calculate_norm(v1)
    v2_norm = calculate_norm(v2)
    dot_product = v1[0] * v2[0] + v1[1] * v2[1]
    cos_theta = dot_product / (v1_norm * v2_norm)
    
    if abs(cos_theta) > 1:
        # Vetores colineares, definir ângulo como 0 ou pi
        if cos_theta > 0:
            theta = 0
        else:
            theta = pi
    else:
        sin_theta = sqrt(1 - cos_theta**2)
        theta = atan2(sin_theta, cos_theta)
    return theta

class Circle(object):
    def __init__(self, center, radius, m=create_identity()):
        self.center = center
        self.radius = radius
        self.set_matrix(m)
    def set_center(self, center):
        self.center = center
    def set_radius(self, radius):
        self.radius = radius
    def set_matrix(self, t):
        self.m = t
        self.invm = inverse(t)
    def contains(self, p):
        p = apply_to_vector(self.invm, [p[0], p[1], 0, 1])
        diff = [p[0] - self.center[0], p[1] - self.center[1]]
        distance = calculate_norm(diff)
        return distance <= self.radius
    def get_center(self):
        return self.center
    def draw(self, mode="fill"):
        glPushMatrix()
        glMultMatrixf(self.m)

        num_segments = 50 # Número de segmentos para aproximar o círculo
        angle_step = 2 * pi / num_segments

        if mode == "outline":
            glBegin(GL_LINE_LOOP)
            for i in range(num_segments):
                angle = i * angle_step
                x = self.radius * cos(angle) + self.center[0]
                y = self.radius * sin(angle) + self.center[1]
                glVertex2f(x, y)
            glEnd()
        else:
            glBegin(GL_TRIANGLE_FAN)
            glVertex2f(self.center[0], self.center[1])  # Vértice central do círculo
            for i in range(num_segments + 1):  # +1 para fechar o círculo
                angle = i * angle_step
                x = self.radius * cos(angle) + self.center[0]
                y = self.radius * sin(angle) + self.center[1]
                glVertex2f(x, y)
            glEnd()

        glPopMatrix()

class Rect(object):
    def __init__ (self, points, m = create_identity()):
        self.points = points
        self.set_matrix(m)
    def set_point (self, i, p):
        self.points[i] = p
    def set_matrix(self,t):
        self.m = t
        self.invm = inverse(t)
    def contains(self,p):
        p = apply_to_vector(self.invm, [p[0],p[1],0,1])
        xmin = min(self.points[0][0],self.points[1][0])
        xmax = max(self.points[0][0],self.points[1][0])
        ymin = min(self.points[0][1],self.points[1][1])
        ymax = max(self.points[0][1],self.points[1][1])
        return xmin <= p[0] <= xmax and ymin <=p[1] <= ymax
    def get_center(self):
        x = (self.points[0][0] + self.points[1][0]) / 2
        y = (self.points[0][1] + self.points[1][1]) / 2
        return (x, y)
    def draw (self):
        glPushMatrix()
        glMultMatrixf(self.m)
        glRectf(*self.points[0],*self.points[1])
        glPopMatrix()

picked = None
modeConstants = ["CREATE REACT", "CREATE CIRCLE", "TRANSLATE", "ROTATE", "SCALE"]
mode = modeConstants[0]

def reshape( width, height):
    glViewport(0,0,width,height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluOrtho2D(0,width,height,0)
    glMatrixMode (GL_MODELVIEW)

def mouse (button, state, x, y):
    global lastx,lasty,picked
    if state!= GLUT_DOWN: return
    if mode == "CREATE REACT":
        shapes.append(Rect([[x,y],[x,y]]))
    elif mode == "CREATE CIRCLE":
        shapes.append(Circle([x,y], 0))
    elif mode == "TRANSLATE" or mode == "ROTATE" or mode == "SCALE":
        picked = None
        for s in shapes:
            if s.contains([x,y]): picked = s
        lastx,lasty = x,y
    """elif mode == "SCALE":
        picked = None
        for s in shapes:
            if s.contains([x,y]):
                print("picked")
                picked = s
                center = picked.get_center()
                vec_mouse = [x - center[0], y - center[1]]
                
                # Calcula o ângulo entre o vetor vec_mouse e o eixo x
                theta = angle_between_vectors(center, [1, 0, 0])
                theta_x_axis = angle_between_vectors(vec_mouse, [1, 0, 0])

                print(theta)
                print(theta_x_axis)
                
                # Calcula o fator de escala dx ao longo do vetor vec_mouse
                dx = 100
                
                # Cria as transformações necessárias
                t1 = create_from_translation([-center[0], -center[1], 0])
                t2 = create_from_x_rotation(-theta_x_axis)
                t3 = create_from_scale([1 + dx * 0.001, 1, 1])
                print(theta_x_axis)
                t4 = create_from_x_rotation(theta_x_axis - 0.16)
                t5 = create_from_translation([center[0], center[1], 0])

                # Multiplica as transformações na ordem correta
                t = multiply(t4, t5)
                t = multiply(t3, t)
                t = multiply(t2, t)
                t = multiply(t1, t)
                
                picked.set_matrix(multiply(picked.m, t))"""

def mouse_drag(x, y):
    global lastx, lasty
    if mode == "CREATE REACT":
        shapes[-1].set_point(1,[x,y])
    elif mode == "CREATE CIRCLE":
        s = shapes[-1]
        diff = [x - s.center[0], y - s.center[1]]
        norm_diff = calculate_norm(diff)
        s.set_radius(norm_diff)
    elif mode == "TRANSLATE":
        if picked:
            t = create_from_translation([x-lastx,y-lasty,0])
            picked.set_matrix(multiply(picked.m,t))
            lastx,lasty=x,y
    elif mode == "ROTATE":
        if picked:
            center = picked.get_center()
            vec_mouse = [lastx - center[0], lasty - center[1]]
            vec_mouse_drag = [x - center[0], y - center[1]]
            theta = angle_between_vectors(vec_mouse, vec_mouse_drag)

            # Determinar o sentido da rotação com base no ângulo entre os vetores
            if vec_mouse[0] * vec_mouse_drag[1] - vec_mouse[1] * vec_mouse_drag[0] > 0:
                theta *= -1  # Inverter o sentido da rotação

            t1 = create_from_translation([-center[0], -center[1], 0])
            t2 = create_from_z_rotation(theta)
            t3 = create_from_translation([center[0], center[1], 0])

            t = multiply(t2, t3)
            t = multiply(t1, t)
            picked.set_matrix(multiply(picked.m, t))

            lastx, lasty = x, y
    elif mode == "SCALE":
        if picked:
            """center = picked.get_center()
            vec_mouse = [lastx - center[0], lasty - center[1]]
            vec_mouse_drag = [x - lastx, y - lasty]
            theta = angle_between_vectors(vec_mouse, vec_mouse_drag)
           
            d = calculate_norm([vec_mouse_drag[0] - vec_mouse[0], vec_mouse_drag[1] - vec_mouse[1]]) * cos(theta)
            theta_x_axis = angle_between_vectors(vec_mouse, [1, 0, 0])
            dx = d * cos(theta_x_axis)
            #dy = d * sin(theta_x_axis)
            theta_y_axis = angle_between_vectors(vec_mouse, [0, 1, 0])

            t1 = create_from_translation([-center[0], -center[1], 0])
            t2 = create_from_x_rotation(-theta_x_axis)

            t3 = create_from_scale([1.0 + dx * 0.001,1.0 + dy * 0.001,1])
            
            t4 = create_from_x_rotation(theta_x_axis)
            t5 = create_from_translation([center[0], center[1], 0])
            
            t = multiply(t4, t5)
            t = multiply(t3, t)
            t = multiply(t2, t)
            t = multiply(t1, t)

            t1 = create_from_translation([-center[0], -center[1], 0])
            t2 = create_from_scale([1.0 + dx * 0.001,1.0 + dy * 0.001,1])
            t3 = create_from_translation([center[0], center[1], 0])

            t = multiply(t2, t3)
            t = multiply(t1, t)"""
            
            """center = picked.get_center()
            vec_mouse = [lastx - center[0], lasty - center[1]]
            vec_mouse_drag = [x - lastx, y - lasty]

            theta = angle_between_vectors(vec_mouse, vec_mouse_drag)
            d = calculate_norm([vec_mouse_drag[0] - vec_mouse[0], vec_mouse_drag[1] - vec_mouse[1]]) 
            theta_x_axis = angle_between_vectors(vec_mouse, [1, 0, 0])
            dx = abs(d * cos(theta_x_axis)) * 0.0001
            

            t1 = create_from_translation([-center[0], -center[1], 0])
            
            t2 = create_from_x_rotation(-theta_x_axis)
            t3 = create_from_scale([1 + dx,1,1])
            t4 = create_from_x_rotation(theta_x_axis)
            
            theta_y_axis = angle_between_vectors(vec_mouse, [0, -1, 0])
            dy = abs(d * cos(theta_y_axis)) * 0.0001
            t5 = create_from_y_rotation(-theta_y_axis)
            t6 = create_from_scale([1, 1 + dy, 1])
            t7 = create_from_y_rotation(theta_y_axis)

            t8 = create_from_translation([center[0], center[1], 0])

            t = multiply(t7, t8)
            t = multiply(t6, t)
            t = multiply(t5, t)
            t = multiply(t4, t)
            t = multiply(t3, t)
            t = multiply(t2, t)
            t = multiply(t1, t)

            picked.set_matrix(multiply(picked.m,t))
            lastx,lasty=x,y"""
            center = picked.get_center()
            vec_mouse = [lastx - center[0], lasty - center[1]]
            vec_mouse_drag = [x - lastx, y - lasty]

            # Calcula o ângulo de rotação necessário para alinhar vec_mouse com o eixo x
            theta = angle_between_vectors(vec_mouse, [1,0,0])

            # Calcula a distância entre o centro e o mouse
            distance = calculate_norm(vec_mouse)

            scale_factor = 0.0001  # Fator de escala (ajuste conforme necessário)
            scale_factor *= distance  # Ajuste do fator de escala com base na distância

            t1 = create_from_translation([-center[0], -center[1], 0])

            t2 = create_from_z_rotation(theta)
            t3 = create_from_scale([1 + scale_factor, 1, 1])
            t4 = create_from_z_rotation(-theta)

            t5 = create_from_translation([center[0], center[1], 0])

            print(theta)
            # Passo 1: Transladar para a origem
            t6 = create_from_translation([-center[0], -center[1], 0])
            angle = radians(-theta)  # Converter o ângulo para radianos
            t7 = create_from_z_rotation(angle)
            t8 = create_from_translation([center[0], center[1], 0])

            t = multiply(t7, t8)
            t = multiply(t6, t)
            t = multiply(t5, t)
            t = t5
            t = multiply(t4, t)
            t = multiply(t3, t)
            t = multiply(t2, t)
            t = multiply(t1, t)

            picked.set_matrix(multiply(picked.m, t))
            lastx, lasty = x, y
    glutPostRedisplay()

def display():
    glClear(GL_COLOR_BUFFER_BIT)
    for s in shapes:
        glColor3f(0.4,0.4,0.4)
        glPolygonMode(GL_FRONT_AND_BACK,GL_FILL)
        s.draw()

        # desenha contorno
        glColor3f(1,0,1)
        if isinstance(s, Circle):
            s.draw("outline")
        else:
            glPolygonMode(GL_FRONT_AND_BACK,GL_LINE)
            s.draw()
    glutSwapBuffers()

def createMenu():
    def domenu(item):
        global mode
        mode = modeConstants[item]
        return 0
    glutCreateMenu(domenu)
    for i,name in enumerate(modeConstants):
        glutAddMenuEntry(name, i)
    glutAttachMenu(GLUT_RIGHT_BUTTON)

glutInit(sys.argv);
glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB);
glutInitWindowSize (800, 600);
glutCreateWindow ("rectangle editor")
glutMouseFunc(mouse)
glutMotionFunc(mouse_drag)
glutDisplayFunc(display); 
glutReshapeFunc(reshape)
createMenu()
glutMainLoop();
