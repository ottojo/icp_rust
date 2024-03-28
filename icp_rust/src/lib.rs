use kd_tree::{KdPoint, KdTree3};
use log::debug;
use nalgebra as na;
pub mod util;

pub type Point3 = na::Point3<f32>;
pub type Iso = na::Isometry3<f32>;

fn center_of_mass(points: &[Point3]) -> Point3 {
    Point3::from(
        points
            .iter()
            .map(|p| p.coords)
            .fold(na::Vector3::new(0.0, 0.0, 0.0), |a, b| a + b)
            / points.len() as f32,
    )
}

struct PointWithIndex {
    point: Point3,
    index: usize,
}

impl KdPoint for PointWithIndex {
    type Scalar = f32;
    type Dim = typenum::U3;
    fn at(&self, k: usize) -> f32 {
        self.point[k]
    }
}

/// Returns correspondence pairs (i, j) where i is the map index and j is the scan index.
fn calculate_correspondences(
    points_map: &KdTree3<PointWithIndex>,
    points_scan: &[Point3],
    scan_transformation: &Iso,
) -> Vec<(usize, usize)> {
    points_scan
        .iter()
        .enumerate()
        .map(|(j, scan_point)| {
            (
                points_map
                    .nearest(&scan_transformation.transform_point(scan_point))
                    .unwrap()
                    .item
                    .index,
                j,
            )
        })
        .collect()
}

fn registration_error(
    points_map: &[Point3],
    points_scan: &[Point3],
    scan_transformation: &Iso,
    correspondences: &[(usize, usize)],
) -> f32 {
    //let correspondences = correspondences(points_kdtree, points_scan, scan_transformation);
    let len = correspondences.len() as f32;

    correspondences
        .iter()
        .map(|&(i, j)| {
            (points_map[i].coords - scan_transformation.transform_point(&points_scan[j]).coords)
                .norm()
        })
        .sum::<f32>()
        / len
}

pub struct ICPParameters<'a> {
    pub rec: Option<&'a rerun::RecordingStream>,
    pub termination_threshold: f32,
}

impl<'a> Default for ICPParameters<'a> {
    fn default() -> Self {
        Self {
            rec: None,
            termination_threshold: 0.01,
        }
    }
}

/// Returns transformation which must be applied to scan to match map
pub fn icp(
    points_map: &[Point3],
    points_scan: &[Point3],
    initial_guess: na::Isometry3<f32>,
    parameters: &ICPParameters,
) -> na::Isometry3<f32> {
    let mut current_best_transformation = initial_guess;

    let center_of_mass_map = center_of_mass(points_map);
    let center_of_mass_scan = center_of_mass(points_scan);

    let kdtree_map = kd_tree::KdTree::build_by_ordered_float(
        points_map
            .iter()
            .enumerate()
            .map(|(i, p)| PointWithIndex {
                point: *p,
                index: i,
            })
            .collect(),
    );

    let mut correspondences =
        calculate_correspondences(&kdtree_map, points_scan, &current_best_transformation);

    loop {
        let cross_covariance_matrix: na::Matrix3<f32> = correspondences
            .into_iter()
            .map(|(i, j)| {
                (points_map[i] - center_of_mass_map.coords).coords
                    * current_best_transformation
                        .transform_point(&(points_scan[j] - center_of_mass_scan.coords))
                        .coords
                        .transpose()
            })
            .sum();

        let svd = cross_covariance_matrix.svd(true, true);
        let rotation = svd.u.unwrap() * svd.v_t.unwrap();

        // p_transformed = R * (p - center_scan) + center_map
        //               = R * p + (-R * center_scan + center_map)
        //               = R * p + t_isometry

        current_best_transformation *= na::Isometry3::from_parts(
            na::Translation::from(
                -rotation
                    * current_best_transformation
                        .transform_point(&center_of_mass_scan)
                        .coords
                    + center_of_mass_map.coords,
            ),
            na::UnitQuaternion::from_matrix(&rotation),
        );

        if let Some(rec) = parameters.rec {
            rec.log(
                "icp/scan",
                &util::na_iso3_to_rerun_tf(&current_best_transformation),
            )
            .unwrap()
        };

        // Update correspondences with new transformation for error calculation and next iteration
        correspondences =
            calculate_correspondences(&kdtree_map, points_scan, &current_best_transformation);

        let registration_error = registration_error(
            points_map,
            points_scan,
            &current_best_transformation,
            &correspondences,
        );
        debug!("Registration error {}", registration_error);
        if registration_error < parameters.termination_threshold {
            debug!(
                "Registration error {} is less than threshold {}, stopping.",
                registration_error, parameters.termination_threshold
            );
            break;
        }
    }

    current_best_transformation
}
