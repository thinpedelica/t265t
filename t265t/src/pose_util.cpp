#include "pose_util.h"

rs2_pose PoseUtil::identity_pose() {
    rs2_pose pose;
    pose.translation.x = 0;
    pose.translation.y = 0;
    pose.translation.z = 0;
    pose.rotation.x = 0;
    pose.rotation.y = 0;
    pose.rotation.z = 0;
    pose.rotation.w = 1;
    return pose;
}

rs2_pose PoseUtil::reset_object_pose(const rs2_pose& device_pose_in_world) {
    rs2_pose object_pose_in_device;
    object_pose_in_device.translation.x = 0;
    object_pose_in_device.translation.y = 0;
    object_pose_in_device.translation.z = -0.50;
    object_pose_in_device.rotation.x = 0;
    object_pose_in_device.rotation.y = 0;
    object_pose_in_device.rotation.z = 0;
    object_pose_in_device.rotation.w = 1;

    rs2_pose object_pose_in_world = pose_multiply(device_pose_in_world, object_pose_in_device);
    return object_pose_in_world;
}

rs2_pose PoseUtil::pose_inverse(const rs2_pose& p) {
    rs2_pose i;
    i.rotation = quaternion_conjugate(p.rotation);
    i.translation = vector_negate(quaternion_rotate_vector(i.rotation, p.translation));
    return i;
}

rs2_pose PoseUtil::pose_multiply(const rs2_pose& ref2_in_ref1, const rs2_pose& ref3_in_ref2) {
    rs2_pose ref3_in_ref1;
    ref3_in_ref1.rotation = quaternion_multiply(ref2_in_ref1.rotation, ref3_in_ref2.rotation);
    ref3_in_ref1.translation = vector_addition(quaternion_rotate_vector(ref2_in_ref1.rotation, ref3_in_ref2.translation), ref2_in_ref1.translation);
    return ref3_in_ref1;
}

rs2_quaternion PoseUtil::quaternion_conjugate(const rs2_quaternion& q) {
    return rs2_quaternion { -q.x, -q.y, -q.z, q.w };
}

rs2_quaternion PoseUtil::quaternion_multiply(const rs2_quaternion& a, const rs2_quaternion& b) {
    return rs2_quaternion {
        a.x * b.w + a.w * b.x - a.z * b.y + a.y * b.z,
        a.y * b.w + a.z * b.x + a.w * b.y - a.x * b.z,
        a.z * b.w - a.y * b.x + a.x * b.y + a.w * b.z,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    };
}

rs2_vector PoseUtil::quaternion_rotate_vector(const rs2_quaternion& q, const rs2_vector& v) {
    rs2_quaternion v_as_quaternion = { v.x, v.y, v.z, 0 };
    rs2_quaternion rotated_v = quaternion_multiply(quaternion_multiply(q, v_as_quaternion), quaternion_conjugate(q));
    return rs2_vector { rotated_v.x, rotated_v.y, rotated_v.z };
}

rs2_vector PoseUtil::pose_transform_point(const rs2_pose& pose, const rs2_vector& p) {
    return vector_addition(quaternion_rotate_vector(pose.rotation, p), pose.translation);
}

rs2_vector PoseUtil::vector_addition(const rs2_vector& a, const rs2_vector& b) {
    return rs2_vector { a.x + b.x, a.y + b.y, a.z + b.z };
}

rs2_vector PoseUtil::vector_negate(const rs2_vector& v) {
    return rs2_vector { -v.x, -v.y, -v.z };
}
