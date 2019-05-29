
import tf as transf
import math
import numpy
from copy import deepcopy
import hashlib


def transform_to_world(part_transform, camera_transform):
    """
        Uses the part transform and the camera transform to calculate the part position in world's coordinates
        part_transform - Transform object of the part frame
        camera_transform - Transform object of the camera frame

        return coordinates, angles
    """
    result = camera_transform.get_transform_matrix().dot(
        part_transform.get_transform_matrix())
    scale, shear, angles, translate, perspective = transf.transformations.decompose_matrix(
        result)
    # print "---------------- Matrix Composed --------------"
    # print result
    # print "---------------- Matrix Composed --------------"
    # print "---------------- Translation and rotation --------------"
    # print translate, angles
    # print "---------------- Translation and rotation --------------"
    return translate, angles


def transform_list_to_world(transforms):
    """
    transforms must be a list of transform objects, in which the world transform must be the first and the point to be parsed the last

    return coordinates, angles

    """
    # print "---------------- Transform Composed --------------"
    # print transforms
    # print "---------------- Transform Composed --------------"    
    lastTf = None
    resultTf = None
    while len(transforms) > 0:
        lastTf = transforms.pop(0)['transform']
        if resultTf is None:
            resultTf = lastTf.get_transform_matrix()
        else:
            resultTf = lastTf.get_transform_matrix().dot(resultTf)

    # print "---------------- Matrix Composed --------------"
    # print resultTf
    # print "---------------- Matrix Composed --------------"
    scale, shear, angles, translate, perspective = transf.transformations.decompose_matrix(
        resultTf)
    # print "---------------- Translation and rotation --------------"
    # print translate, angles
    # print "---------------- Translation and rotation --------------"
    return translate, angles


def transformDistance(transform, distance):
    #rospy.loginfo("translation %s" % pos.translation)
    #rospy.loginfo("rotation %s" % pos.rotation)
    # var_pos = Vec3(pos.translation.x, pos.translation.y, pos.translation.z)
    # var_rot = Quat(pos.rotation.x, pos.rotation.y,
    #                pos.rotation.z, pos.rotation.w)

    rotate_vec = deepcopy(transform)
    point = numpy.array((0, 0, distance, 1), dtype=numpy.float64)

    # t = Transform(var_pos, var_rot)
    # point = Vec3(0, 0, distance)

    p = rotate_vec.transform_vec_in_place(point)

    rotate_vec.translation[0] = p[0]
    rotate_vec.translation[1] = p[1]
    rotate_vec.translation[2] = p[2]

    return rotate_vec


class Transform(object):
    """ 
        Transform Object 
    """

    def __init__(self, translation=None, rot=None):

        self.translation = numpy.empty((3, ), dtype=numpy.float64)

        self.rot = numpy.empty((4, ), dtype=numpy.float64)

        # translation to be used in matrix operations
        self.x4_translation = numpy.empty((4, ), dtype=numpy.float64)

        if isinstance(translation, list):
            self.translation[0] = translation[0]
            self.translation[1] = translation[1]
            self.translation[2] = translation[2]

        elif isinstance(translation, dict):
            self.translation[0] = translation['x']
            self.translation[1] = translation['y']
            self.translation[2] = translation['z']

        if isinstance(rot, list):
            if len(rot) == 3:
                self.rot = transf.transformations.quaternion_from_euler(rot[0], rot[1], rot[2])
            elif len(rot) == 4:
                self.rot[0] = rot[0]
                self.rot[1] = rot[1]
                self.rot[2] = rot[2]
                self.rot[3] = rot[3]

        elif isinstance(rot, dict):
            if len(rot.keys()) == 3:
                self.rot = transf.transformations.quaternion_from_euler(
                    [rot['x'], rot['y'], rot['z']])
            elif len(rot.keys()) == 4:
                self.rot[0] = rot['x']
                self.rot[1] = rot['y']
                self.rot[2] = rot['z']
                self.rot[3] = rot['w']

        # quaternion as a matrix
        self.rot_matrix = transf.transformations.quaternion_matrix(self.rot)

        # transpost of the quaternion matrix
        self.t_rot_matrix = numpy.transpose(self.rot_matrix)

        self.x4_translation[0] = self.translation[0]
        self.x4_translation[1] = self.translation[1]
        self.x4_translation[2] = self.translation[2]
        self.x4_translation[3] = 1

    def get_transform_matrix(self):
        """
            Create the transform matrix from the quaternion and the translation.

            Transform Matrix:
            R R R T
            R R R T
            R R R T
            0 0 0 1
        """
        transform_matrix = numpy.array(self.rot_matrix, copy=True)
        transform_matrix[0][3] = self.translation[0]
        transform_matrix[1][3] = self.translation[1]
        transform_matrix[2][3] = self.translation[2]

        return transform_matrix

    def get_inverse_transform_matrix(self):
        """
            Create the inverse transform matrix from the quaternion and the translation.
            Inverse Transform matrix:
            R^t R^t R^t R^t*T
            R^t R^t R^t R^t*T
            R^t R^t R^t R^t*T
             0   0   0    1
        """
        return numpy.linalg.inv(self.get_transform_matrix())

    def __str__(self):
        return "translation:[" + str(",".join([str(v) for v in self.translation])) + "], rotation:[" + str(",".join([str(v) for v in self.rot])) + "]"

    def hash_value(self):
        return hashlib.sha224(self.__str__()).hexdigest()

    def transform_vec_in_place(self, point):
        # New function, testing.
        nv = self.rotate_vec(point)
         
        return numpy.array((nv[0] + self.translation[0], nv[1] + self.translation[1], nv[2] + self.translation[2]), dtype=numpy.float64)

    def rotate_vec(self, vec):
        """
        https://code.google.com/p/kri/wiki/Quaternions
         v + 2.0*cross(q.xyz, cross(q.xyz,v) + q.w*v);
        """
        xyz1 = numpy.array((vec[0], vec[1], vec[2], 1), dtype=numpy.float64)

        return self.rot_matrix.dot(xyz1)




# from transform import *
# father: logical_camera_agv_1_frame; child: logical_camera_agv_1_gear_part_7_frame
# father: world; child: logical_camera_agv_1_frame
# translation:[0.746758806387,-0.00503701637988,0.101087234161], rotation:[0.00905606441507,-0.708537479154,-0.00372872092838,0.705605218923]
# translation:[0.3,3.15,1.5], rotation:[0.0,0.707108079859,0.0,0.707105482511]

part_rot_quart = [0.0140612963397, -0.709402022327, -0.00735706574966, 0.704625378657]
part_rot_euler = transf.transformations.euler_from_quaternion(part_rot_quart)
part_translate = [0.746795254399,-0.00440293764398, 0.100420057159]


camera_rot_quart = [0.0,0.707108079859,0.0,0.707105482511]
camera_rot_euler = transf.transformations.euler_from_quaternion(camera_rot_quart)
camera_translate = [0.3,3.15,1.5]

part = Transform(part_translate, part_rot_quart)
camera = Transform(camera_translate, camera_rot_quart)

camera_matrix = transf.transformations.compose_matrix(scale=None, shear=None, angles=camera_rot_euler, translate=camera_translate, perspective=None)
part_matrix = transf.transformations.compose_matrix(scale=None, shear=None, angles=part_rot_euler, translate=part_translate, perspective=None)


