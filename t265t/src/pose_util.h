#include <rs.hpp>
#include <rsutil.h>

class PoseUtil {
public:
    static rs2_pose identity_pose();
    static rs2_pose reset_object_pose(const rs2_pose& device_pose_in_world = identity_pose());
    static rs2_pose pose_inverse(const rs2_pose& p);
    static rs2_pose pose_multiply(const rs2_pose& ref2_in_ref1, const rs2_pose& ref3_in_ref2);
    static rs2_quaternion quaternion_conjugate(const rs2_quaternion& q);
    static rs2_quaternion quaternion_multiply(const rs2_quaternion& a, const rs2_quaternion& b);
    static rs2_vector quaternion_rotate_vector(const rs2_quaternion& q, const rs2_vector& v);
    static rs2_vector pose_transform_point(const rs2_pose& pose, const rs2_vector& p);
    static rs2_vector vector_addition(const rs2_vector& a, const rs2_vector& b);
    static rs2_vector vector_negate(const rs2_vector& v);

};
