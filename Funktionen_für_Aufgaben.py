import numpy as np


def yaw_pitch_roll(yaw_grad,pitch_grad,roll_grad, p=None):#ZYX sequenz
    #yaw_grad=90
    #pitch_grad=0
    #roll_grad=0

    yaw=np.radians(yaw_grad)
    pitch=np.radians(pitch_grad)
    roll=np.radians(roll_grad)

    #Aufgabe 3.1

    # Z-achse
    sy=np.sin(yaw)
    cy=np.cos(yaw)

    rz=np.array([
        [cy,-sy,0],
        [sy, cy,0],
        [0,  0, 1]
    ])


    # Y-achse
    sp=np.sin(pitch)
    cp=np.cos(pitch)
    ry=np.array([
        [ cp, 0 , sp],
        [ 0 , 1 , 0 ],
        [-sp, 0 , cp]
    ])

    # X-achse
    sr=np.sin(roll)
    cr=np.cos(roll)


    rx=np.array([
        [1, 0,  0],
        [0,cr,-sr],
        [0,sr, cr]
    ])


    # Aufgabe 3.2 multiplikation rz*ry*rx


    r=rz@ry@rx
    
    if p is None:
        return r
    else:
        p_new=r@p
        return p_new


    




def cosinus_satz(ak, bk, gk):

    cos_gamma = (ak**2 + bk**2 - gk**2) / (2 * ak * bk)
    cos_gamma = np.clip(cos_gamma, -1.0, 1.0)

    gamma = np.arccos(cos_gamma)

    return gamma



def DH_Matrix (alpha,a,theta,d):
    c_theta=np.cos(theta)
    s_theta=np.sin(theta)
    c_alpha=np.cos(alpha)
    s_alpha=np.sin(alpha)

    T=np.array([
        [c_theta,-s_theta,0,a],
        [c_alpha*s_theta,c_alpha*c_theta,-s_alpha,-d*s_alpha],
        [s_alpha*s_theta,s_alpha*c_theta,c_alpha,d*c_alpha],
        [0,0,0,1]
    ])
    return T


def forward_kinematics(DH_params):
    """
    Compute the forward kinematics for the robot.
    """
    T = np.eye(4)
    frames = [T]
    for i in range(DH_params.shape[0]):
        T = T @ DH_Matrix(*DH_params[i])
        frames.append(T)
    return frames


class Vektor:

    def __init__(self, v):
        self.v = np.array(v)

    def __str__(self):
        return f"Vektor{tuple(self.v)}"

    @property
    def betrag(self):
        return np.linalg.norm(self.v)

    @property
    def einheitsvektor(self):
        return self.v / self.betrag

    def skalarprodukt(self, other):
        return np.dot(self.v, other.v)

    def kreuzprodukt(self, other):
        return np.cross(self.v, other.v)

    def winkel(self, other):
        cos_phi = self.skalarprodukt(other) / (
            self.betrag * other.betrag
        )
        cos_phi = np.clip(cos_phi, -1.0, 1.0)
        return np.rad2deg(np.arccos(cos_phi))
    
    def info(self):
        print("=== Vektorinfo ===")
        print(f"Vektor         : {self.v}")
        print(f"Betrag         : {self.betrag:.4f}")
        print(f"Einheitsvektor : {self.einheitsvektor}")


def XYZ_RotationsMatrix(achse,winkel):

    s=np.sin(winkel)
    c=np.cos(winkel)
    

    if achse=='Y' or achse =='y':
       R= np.array([
           [c,0,s],
           [0,1,0],
           [-s,0,c]
       ])
       return R
    

    if achse=='X'or achse=='x':
        R=np.array([
            [1,0,0],
            [c,-s,0],
            [s,c,0]
        ])
        return R

    if achse=='Z' or achse =='z':
        R=np.array([
            [c,-s,0],
            [s,c,0],
            [0,0,1]
        ])
        return R
    
def Transformationsmatrix(achse,winkel,vektor):
    R=XYZ_RotationsMatrix(achse,winkel)
    
    h=np.array([[0,0,0]])
    t=np.vstack([R,h])
    T=np.hstack([t,vektor])
    return(T)


def invers_transformationsmatrix(T):
    R = T[:3, :3]
    h=np.array([0,0,0,1])
    v=T[:3, 3:]
    iR=np.linalg.inv(R)
    iv=-iR@v
    it=np.hstack([iR,iv])
    iT=np.vstack([it,h])
    
    
    return(iT)
    