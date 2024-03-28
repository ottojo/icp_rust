use nalgebra as na;
use rerun::external::glam;

pub fn glam_vec_to_na_point(v: glam::Vec3) -> na::Point3<f32> {
    na::Point3::new(v.x, v.y, v.z)
}

pub fn na_point_to_glam_vec(p: na::Point3<f32>)->glam::Vec3{
    glam::Vec3::new(p.x, p.y, p.z)
}