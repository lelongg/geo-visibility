use crate::utils::cross;

#[derive(Debug, Clone, PartialEq)]
pub enum Orientation {
    LeftTurn,
    RightTurn,
    Collinear,
}

impl Orientation {
    pub fn from(a: geo::Point<f64>, b: geo::Point<f64>, c: geo::Point<f64>) -> Self {
        match cross(b - a, c - a) {
            det if det > 0.0 => Orientation::LeftTurn,
            det if det < 0.0 => Orientation::RightTurn,
            _ => Orientation::Collinear,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo::point;

    #[test]
    fn test_orientation() {
        assert_eq!(
            Orientation::from(
                point!(x: 0.0, y: 0.0),
                point!(x: 1.0, y: 0.0),
                point!(x: 2.0, y: 1.0)
            ),
            Orientation::LeftTurn
        );
        assert_eq!(
            Orientation::from(
                point!(x: 0.0, y: 0.0),
                point!(x: 1.0, y: 0.0),
                point!(x: 2.0, y: -1.0)
            ),
            Orientation::RightTurn
        );
        assert_eq!(
            Orientation::from(
                point!(x: 0.0, y: 0.0),
                point!(x: 1.0, y: 0.0),
                point!(x: 2.0, y: 0.0)
            ),
            Orientation::Collinear
        );
        assert_eq!(
            Orientation::from(
                point!(x: 0.0, y: 0.0),
                point!(x: 0.0, y: 0.0),
                point!(x: 4.0, y: 5.0)
            ),
            Orientation::Collinear
        );
        assert_eq!(
            Orientation::from(
                point!(x: 0.0, y: 0.0),
                point!(x: 0.0, y: 0.0),
                point!(x: 0.0, y: 0.0)
            ),
            Orientation::Collinear
        );
    }
}
