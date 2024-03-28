use nalgebra as na;
use rerun::external::glam;

pub fn glam_vec_to_na_point(v: glam::Vec3) -> na::Point3<f32> {
    na::Point3::new(v.x, v.y, v.z)
}

pub fn na_point_to_glam_vec(p: na::Point3<f32>) -> glam::Vec3 {
    glam::Vec3::new(p.x, p.y, p.z)
}

pub fn na_iso3_to_rerun_tf(iso: &na::Isometry3<f32>) -> rerun::archetypes::Transform3D {
    let t = iso.translation;
    let r = iso.rotation.coords;

    rerun::Transform3D::from_translation_rotation(
        glam::Vec3::new(t.x, t.y, t.z),
        rerun::Quaternion([r.x, r.y, r.z, r.w]),
    )
}
