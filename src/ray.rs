use crate::{orientation::Orientation, utils::cross};

#[derive(Debug, Clone, PartialEq)]
pub struct Ray {
    pub line: geo::Line<f64>,
}

impl Ray {
    pub fn new(line: geo::Line<f64>) -> Self {
        Self { line }
    }

    pub fn intersects(&self, segment: &geo::Line<f64>) -> Option<geo::Point<f64>> {
        let epsilon = 1E-4;
        let origin = geo::Point::from(self.line.start);
        let direction = geo::Point::from(self.line.end) - origin;
        let a = geo::Point::from(segment.start);
        let b = geo::Point::from(segment.end);
        let ao = origin - a;
        let ab = b - a;
        let det = cross(ab, direction);

        if det.abs() < epsilon {
            if Orientation::from(a, b, origin) != Orientation::Collinear {
                None
            } else {
                let dist_a = ao.dot(direction);
                let dist_b = (origin - b).dot(direction);

                if dist_a > 0.0 && dist_b > 0.0 {
                    None
                } else {
                    Some(if (dist_a > 0.0) != (dist_b > 0.0) {
                        origin
                    } else if dist_a > dist_b {
                        a
                    } else {
                        b
                    })
                }
            }
        } else {
            let u = cross(ao, direction) / det;
            if !(0.0..=1.0).contains(&u) {
                None
            } else {
                let t = -cross(ab, ao) / det;
                if t.abs() < epsilon || t > 0.0 {
                    Some(origin + geo::Point::new(direction.x() * t, direction.y() * t))
                } else {
                    None
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo::{point, Coord, Line};

    #[test]
    fn test_ray() {
        assert_eq!(
            Ray::new(Line::new(
                Coord { x: 0.0, y: 0.0 },
                Coord { x: 1.0, y: 0.0 },
            ))
            .intersects(&Line::new(
                Coord { x: -1.0, y: 1.0 },
                Coord { x: -1.0, y: -1.0 },
            )),
            None
        );
        assert_eq!(
            Ray::new(Line::new(
                Coord { x: 0.0, y: 0.0 },
                Coord { x: 1.0, y: 0.0 },
            ))
            .intersects(&Line::new(
                Coord { x: -1E-3, y: 1.0 },
                Coord { x: -1E-3, y: -1.0 },
            )),
            None
        );
        assert_eq!(
            Ray::new(Line::new(
                Coord { x: 0.0, y: 0.0 },
                Coord { x: 1.0, y: 0.0 },
            ))
            .intersects(&Line::new(
                Coord { x: -2.0, y: 0.0 },
                Coord { x: -1.0, y: 0.0 },
            )),
            None
        );
        assert_eq!(
            Ray::new(Line::new(
                Coord { x: 0.0, y: 0.0 },
                Coord { x: 1.0, y: 0.0 },
            ))
            .intersects(&Line::new(
                Coord { x: 0.0, y: 1.0 },
                Coord { x: 0.0, y: -1.0 },
            )),
            Some(point!(x: 0.0, y: 0.0))
        );
        assert_eq!(
            Ray::new(Line::new(
                Coord { x: 0.0, y: 0.0 },
                Coord { x: 1.0, y: 0.0 },
            ))
            .intersects(&Line::new(
                Coord { x: -1.0, y: 0.0 },
                Coord { x: 0.0, y: 0.0 },
            )),
            Some(point!(x: 0.0, y: 0.0))
        );
        assert_eq!(
            Ray::new(Line::new(
                Coord { x: 0.0, y: 0.0 },
                Coord { x: 1.0, y: 0.0 },
            ))
            .intersects(&Line::new(
                Coord { x: 0.0, y: 0.0 },
                Coord { x: -1.0, y: 0.0 },
            )),
            Some(point!(x: 0.0, y: 0.0))
        );
        assert_eq!(
            Ray::new(Line::new(
                Coord { x: 0.0, y: 0.0 },
                Coord { x: 1.0, y: 0.0 },
            ))
            .intersects(&Line::new(
                Coord { x: 2.0, y: 1.0 },
                Coord { x: 2.0, y: -1.0 },
            )),
            Some(point!(x: 2.0, y: 0.0))
        );
        assert_eq!(
            Ray::new(Line::new(
                Coord { x: 0.0, y: 0.0 },
                Coord { x: 1.0, y: 0.0 },
            ))
            .intersects(&Line::new(
                Coord { x: 2.0, y: 0.0 },
                Coord { x: 3.0, y: 0.0 },
            )),
            Some(point!(x: 2.0, y: 0.0))
        );
        assert_eq!(
            Ray::new(Line::new(
                Coord { x: 0.0, y: 0.0 },
                Coord { x: 1.0, y: 0.0 },
            ))
            .intersects(&Line::new(
                Coord { x: 3.0, y: 0.0 },
                Coord { x: 2.0, y: 0.0 },
            )),
            Some(point!(x: 2.0, y: 0.0))
        );
        assert_eq!(
            Ray::new(Line::new(
                Coord { x: 0.5, y: 0.0 },
                Coord { x: 2.0, y: 0.0 },
            ))
            .intersects(&Line::new(
                Coord { x: 1.0, y: 0.0 },
                Coord { x: 1.0, y: -1.0 },
            )),
            Some(point!(x: 1.0, y: 0.0))
        );
    }
}
