use approx::abs_diff_eq;
use geo::{Distance, Euclidean};
 
pub fn cross(a: geo::Point<f64>, b: geo::Point<f64>) -> f64 {
    a.x() * b.y() - a.y() * b.x()
}

pub fn approx_equal(a: &geo::Point<f64>, b: &geo::Point<f64>) -> bool {
    abs_diff_eq!(Euclidean::distance(a, b), 0.0)
}

#[cfg(test)]
#[allow(clippy::float_cmp)]
mod tests {
    use super::*;
    use geo::point;

    #[test]
    fn test_dot() {
        assert_eq!(point!(x: 1.0, y: 2.0).dot(point!(x: 3.0, y: 4.0)), 11.0);
        assert_eq!(point!(x: 1.0, y: 2.0).dot(point!(x: 0.0, y: 0.0)), 0.0);
    }

    #[test]
    fn test_det() {
        assert_eq!(cross(point!(x: 3.0, y: 4.0), point!(x: 1.0, y: 2.0)), 2.0);
    }
}
