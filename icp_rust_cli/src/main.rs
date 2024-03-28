use nalgebra as na;
use pcd_rs::DynReader;
use rerun::external::anyhow;
use rerun::external::glam;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();

    let rec = rerun::RecordingStreamBuilder::new("rerun_test_pc").connect()?;

    let reader = DynReader::open("/home/jonas/Downloads/ism_test_cat.pcd")?;
    let points: anyhow::Result<Vec<_>> = reader.collect();
    let points = points?;

    let points_map: Vec<_> = points
        .into_iter()
        .map(|dr| {
            let xyz: [f32; 3] = dr.to_xyz().unwrap();
            glam::Vec3::new(xyz[0], xyz[1], xyz[2])
        })
        .collect();

    let points_map_na: Vec<_> = points_map
        .iter()
        .cloned()
        .map(icp_rust::util::glam_vec_to_na_point)
        .collect();

    let offset = na::Isometry3::new(
        na::Vector3::new(0.1, 0.7, 0.5),
        na::Vector3::new(0.1, 0.2, 0.3),
    );

    let points_scan: Vec<_> = points_map
        .iter()
        .cloned()
        .map(icp_rust::util::glam_vec_to_na_point)
        .map(|p| offset.transform_point(&p))
        .collect();

    rec.log("icp/map", &rerun::Points3D::new(points_map))?;

    rec.log(
        "icp/scan",
        &rerun::Points3D::new(
            points_scan
                .iter()
                .cloned()
                .map(icp_rust::util::na_point_to_glam_vec),
        ),
    )?;

    let _icp_alignment = icp_rust::icp(
        &points_map_na,
        &points_scan,
        na::Isometry3::identity(),
        &icp_rust::ICPParameters {
            rec: Some(&rec),
            ..Default::default()
        },
    );

    Ok(())
}
