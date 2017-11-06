
import geometry_msgs
import tf.transformations as ttf
import numpy as np

def calcRelativeCoordinate(pose, point): # (geometry_msgs::Point point_msg, geometry_msgs::Pose current_pose)
    worldCarR_q = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    worldCarT_v = np.array([pose.position.x, pose.position.y, pose.position.z, 1.0])

    carWorldR_m = ttf.quaternion_matrix(ttf.quaternion_conjugate(worldCarR_q))
    carWorldT_v = -np.dot(carWorldR_m,worldCarT_v)
    carWorld_m = carWorldR_m
    carWorld_m[:3, 3] = carWorldT_v[:3]
    point_world = np.array([point.x, point.y, point.z, 1.0])
    point_car = np.dot(carWorld_m,point_world)
    p = geometry_msgs.msg.Point()
    p.x, p.y, p.z, _ = point_car
    # print(t)
    # print(p)
    # print(tf2_geometry_msgs.do_transform_point(p, t))
    # assert False ," hh"
    return p


def wrap_calcRelativeCoordinate(position, orientation):
    c_pose = geometry_msgs.msg.Pose()
    c_pose.position.x = position[0]
    c_pose.position.y = position[1]
    c_pose.position.z = position[2]
    c_pose.orientation.w = orientation[0]
    c_pose.orientation.x = orientation[1]
    c_pose.orientation.y = orientation[2]
    c_pose.orientation.z = orientation[3]
    w_pose = geometry_msgs.msg.Point()
    w_pose.x = 1
    w_pose.y = 0
    w_pose.z = 0
    return calcRelativeCoordinate(c_pose, w_pose)


def test_calcRelativeCoordinate_identity():
    result = wrap_calcRelativeCoordinate([0, 0, 0], [1, 0, 0, 0])
    assert result.x == 1.0
    assert result.y == 0.0
    assert result.z == 0.0


def test_calcRelativeCoordinate_180():
    result = wrap_calcRelativeCoordinate([0, 0, 0], [0, 0, 0, 1])
    print(result.x, result.y, result.z)
    assert abs(result.x - -1.0)<0.001
    assert result.y == 0.0
    assert result.z == 0.0


def test_calcRelativeCoordinate_90():
    result = wrap_calcRelativeCoordinate([0, 0, 0], [0.707, 0, 0, 0.707])
    print(result.x, result.y, result.z)
    assert abs(result.x - 0.0)<0.001
    assert abs(result.y - -1.0)<0.001
    assert result.z == 0.0


def test_calcRelativeCoordinate_1_90():
    result = wrap_calcRelativeCoordinate([1, 0, 0], [0.707, 0, 0, 0.707])
    print(result.x, result.y, result.z)
    assert abs(result.x - 0.0) < 0.001
    assert abs(result.y - 0.0) < 0.001
    assert result.z == 0.0


def isInFront(cpose, wpose):
    return 0 < calcRelativeCoordinate(cpose, wpose.position).x
