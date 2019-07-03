use crate::utils::cross;
use approx::abs_diff_eq;
use geo::algorithm::euclidean_distance::EuclideanDistance;

#[derive(Debug, Clone, PartialEq)]
pub struct AngleComparator {
    pub origin: geo::Point<f64>,
}

impl AngleComparator {
    pub fn cmp(&self, a: &geo::Point<f64>, b: &geo::Point<f64>) -> std::cmp::Ordering {
        let is_a_left = a.x() < self.origin.x();
        let is_b_left = b.x() < self.origin.x();
        if is_a_left != is_b_left {
            return if is_b_left {
                std::cmp::Ordering::Less
            } else {
                std::cmp::Ordering::Greater
            };
        }

        if abs_diff_eq!(a.x(), self.origin.x()) && abs_diff_eq!(b.x(), self.origin.x()) {
            return if a.y() >= self.origin.y() || b.y() >= self.origin.y() {
                if b.y() < a.y() {
                    std::cmp::Ordering::Less
                } else {
                    std::cmp::Ordering::Greater
                }
            } else if a.y() < b.y() {
                std::cmp::Ordering::Less
            } else {
                std::cmp::Ordering::Greater
            };
        }

        let oa = *a - self.origin;
        let ob = *b - self.origin;
        let det = cross(oa, ob);

        if abs_diff_eq!(det, 0.0) {
            if a.euclidean_distance(&self.origin) < b.euclidean_distance(&self.origin) {
                std::cmp::Ordering::Less
            } else {
                std::cmp::Ordering::Greater
            }
        } else if det < 0.0 {
            std::cmp::Ordering::Less
        } else {
            std::cmp::Ordering::Greater
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo::{point, Coordinate, Line, Point};

    #[test]
    fn compare_angle_with_two_points_in_general_position() {
        let angle_cmp = AngleComparator {
            origin: point!(x: 0.0, y: 0.0),
        };

        assert_eq!(
            angle_cmp.cmp(&point!(x: 0.0, y: 1.0), &point!(x: 1.0, y: 1.0)),
            std::cmp::Ordering::Less
        );
        assert_ne!(
            angle_cmp.cmp(&point!(x: 1.0, y: 1.0), &point!(x: 0.0, y: 1.0)),
            std::cmp::Ordering::Less
        );

        assert_eq!(
            angle_cmp.cmp(&point!(x: 1.0, y: 1.0), &point!(x: 1.0, y: -1.0)),
            std::cmp::Ordering::Less
        );
        assert_ne!(
            angle_cmp.cmp(&point!(x: 1.0, y: -1.0), &point!(x: 1.0, y: 1.0)),
            std::cmp::Ordering::Less
        );

        assert_eq!(
            angle_cmp.cmp(&point!(x: 1.0, y: 0.0), &point!(x: -1.0, y: -1.0)),
            std::cmp::Ordering::Less
        );
        assert_ne!(
            angle_cmp.cmp(&point!(x: -1.0, y: -1.0), &point!(x: 1.0, y: 0.0)),
            std::cmp::Ordering::Less
        );

        assert_eq!(
            angle_cmp.cmp(&point!(x: 0.0, y: 1.0), &point!(x: 0.0, y: -1.0)),
            std::cmp::Ordering::Less
        );
        assert_ne!(
            angle_cmp.cmp(&point!(x: 0.0, y: -1.0), &point!(x: 0.0, y: 1.0)),
            std::cmp::Ordering::Less
        );
    }

    #[test]
    fn compare_angle_with_two_points_if_they_are_collinear_with_the_origin() {
        let angle_cmp = AngleComparator {
            origin: point!(x: 0.0, y: 0.0),
        };

        assert_eq!(
            angle_cmp.cmp(&point!(x: 1.0, y: 0.0), &point!(x: 2.0, y: 0.0)),
            std::cmp::Ordering::Less
        );
        assert_ne!(
            angle_cmp.cmp(&point!(x: 2.0, y: 0.0), &point!(x: 1.0, y: 0.0)),
            std::cmp::Ordering::Less
        );

        assert_ne!(
            angle_cmp.cmp(&point!(x: 1.0, y: 0.0), &point!(x: 1.0, y: 0.0)),
            std::cmp::Ordering::Less
        );
        assert_ne!(
            angle_cmp.cmp(&point!(x: 0.0, y: 0.0), &point!(x: 0.0, y: 0.0)),
            std::cmp::Ordering::Less
        );
    }
}
