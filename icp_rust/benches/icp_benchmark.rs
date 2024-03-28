use icp_rust::ICPParameters;
use nalgebra as na;
use pcd_rs::DynReader;
use rerun::external::anyhow;

use criterion::{black_box, criterion_group, criterion_main, Criterion};

pub fn criterion_benchmark(c: &mut Criterion) {
    let reader = DynReader::open("/home/jonas/Downloads/ism_test_cat.pcd").unwrap();
    let points: anyhow::Result<Vec<_>> = reader.collect();
    let points = points.unwrap();

    let points_map_na: Vec<_> = points
        .into_iter()
        .map(|dr| {
            let xyz: [f32; 3] = dr.to_xyz().unwrap();
            na::Point3::new(xyz[0], xyz[1], xyz[2])
        })
        .collect();

    let offset = na::Isometry3::new(
        na::Vector3::new(0.1, 0.2, 0.3),
        na::Vector3::new(0.1, 0.2, 0.3),
    );

    let points_scan: Vec<_> = points_map_na
        .iter()
        .cloned()
        .map(|p| offset.transform_point(&p))
        .collect();

    c.bench_function("align cat", |b| {
        b.iter(|| {
            icp_rust::icp(
                black_box(&points_map_na),
                black_box(&points_scan),
                black_box(na::Isometry3::new(
                    na::Vector3::new(0.0, 0.0, 0.0),
                    na::Vector3::new(0.0, 0.0, 0.0),
                )),
                &ICPParameters {
                    termination_threshold: 0.01,
                    ..Default::default()
                },
            )
        })
    });
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
