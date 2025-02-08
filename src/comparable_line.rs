use crate::{orientation::Orientation, utils::approx_equal};
use geo::{Distance, Euclidean};

#[derive(Debug, Clone, PartialEq)]
pub struct ComparableLine {
    pub origin: geo::Point<f64>,
    pub line: geo::Line<f64>,
}

impl ComparableLine {
    pub fn new(origin: geo::Point<f64>, line: geo::Line<f64>) -> Self {
        Self { origin, line }
    }
}

impl Eq for ComparableLine {}

impl PartialOrd for ComparableLine {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for ComparableLine {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        let (a, b) = (
            geo::Point::from(self.line.start),
            geo::Point::from(self.line.end),
        );
        let (c, d) = (
            geo::Point::from(other.line.start),
            geo::Point::from(other.line.end),
        );

        assert_ne!(
            Orientation::from(self.origin, a, b),
            Orientation::Collinear,
            "segment AB must not be collinear with the origin: {:?}, {:?}",
            self.origin,
            self.line
        );

        assert_ne!(
            Orientation::from(self.origin, c, d),
            Orientation::Collinear,
            "segment CD must not be collinear with the origin: {:?}, {:?}",
            self.origin,
            other.line
        );

        // sort the endpoints so that if there are common endpoints, it will be a and c
        let (a, b) = if approx_equal(&b, &c) || approx_equal(&b, &d) {
            (b, a)
        } else {
            (a, b)
        };
        let (c, d) = if approx_equal(&a, &d) { (d, c) } else { (c, d) };

        // cases with common endpoints
        if approx_equal(&a, &c) {
            return if approx_equal(&b, &d) {
                std::cmp::Ordering::Equal
            } else if Orientation::from(self.origin, a, d) != Orientation::from(self.origin, a, b) {
                std::cmp::Ordering::Greater
            } else if Orientation::from(a, b, d) != Orientation::from(a, b, self.origin) {
                std::cmp::Ordering::Less
            } else {
                std::cmp::Ordering::Greater
            };
        }

        // cases without common endpoints
        let cda = Orientation::from(c, d, a);
        let cdb = Orientation::from(c, d, b);

        if cdb == Orientation::Collinear && cda == Orientation::Collinear {
            if Euclidean::distance(&self.origin,& a) < Euclidean::distance(&self.origin, &c) {
                std::cmp::Ordering::Less
            } else {
                std::cmp::Ordering::Greater
            }
        } else if cda == cdb || cda == Orientation::Collinear || cdb == Orientation::Collinear {
            let cdo = Orientation::from(c, d, self.origin);
            if cdo == cda || cdo == cdb {
                std::cmp::Ordering::Less
            } else {
                std::cmp::Ordering::Greater
            }
        } else if Orientation::from(a, b, self.origin) != Orientation::from(a, b, c) {
            std::cmp::Ordering::Less
        } else {
            std::cmp::Ordering::Greater
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo::{point, Point};

    #[test]
    fn compare_two_line_segments_with_no_common_endpoints() {
        let origin = point!(x: 0.0, y: 0.0);
        test_line_segment_is_closer(
            origin,
            point!(x: 1.0, y: 1.0),
            point!(x: 1.0, y: -1.0),
            point!(x: 2.0, y: 1.0),
            point!(x: 2.0, y: -1.0),
        );
        test_line_segment_is_closer(
            origin,
            point!(x: 1.0, y: 1.0),
            point!(x: 1.0, y: -1.0),
            point!(x: 2.0, y: 2.0),
            point!(x: 2.0, y: 3.0),
        );
    }

    #[test]
    fn compare_two_line_segments_with_common_endpoints() {
        let origin = point!(x: 0.0, y: 0.0);
        test_line_segments_are_equal(
            origin,
            point!(x: 1.0, y: 1.0),
            point!(x: 1.0, y: 0.0),
            point!(x: 1.0, y: 0.0),
            point!(x: 1.0, y: -1.0),
        );
        test_line_segments_are_equal(
            origin,
            point!(x: 1.0, y: 1.0),
            point!(x: 1.0, y: 0.0),
            point!(x: 1.0, y: 0.0),
            point!(x: 1.0, y: 1.0),
        );
        test_line_segment_is_closer(
            origin,
            point!(x: 2.0, y: 0.0),
            point!(x: 1.0, y: 1.0),
            point!(x: 2.0, y: 1.0),
            point!(x: 2.0, y: 0.0),
        );
        test_line_segment_is_closer(
            origin,
            point!(x: 2.0, y: 1.0),
            point!(x: 2.0, y: 0.0),
            point!(x: 2.0, y: 0.0),
            point!(x: 3.0, y: 1.0),
        );
    }

    fn test_line_segment_is_closer(
        origin: Point<f64>,
        a: Point<f64>,
        b: Point<f64>,
        c: Point<f64>,
        d: Point<f64>,
    ) {
        assert_eq!(
            ComparableLine::new(origin, geo::Line::new(a, b))
                .cmp(&ComparableLine::new(origin, geo::Line::new(c, d))),
            std::cmp::Ordering::Less
        );
        assert_eq!(
            ComparableLine::new(origin, geo::Line::new(b, a))
                .cmp(&ComparableLine::new(origin, geo::Line::new(c, d))),
            std::cmp::Ordering::Less
        );
        assert_eq!(
            ComparableLine::new(origin, geo::Line::new(a, b))
                .cmp(&ComparableLine::new(origin, geo::Line::new(d, c))),
            std::cmp::Ordering::Less
        );
        assert_eq!(
            ComparableLine::new(origin, geo::Line::new(b, a))
                .cmp(&ComparableLine::new(origin, geo::Line::new(d, c))),
            std::cmp::Ordering::Less
        );

        assert_ne!(
            ComparableLine::new(origin, geo::Line::new(c, d))
                .cmp(&ComparableLine::new(origin, geo::Line::new(a, b))),
            std::cmp::Ordering::Less
        );
        assert_ne!(
            ComparableLine::new(origin, geo::Line::new(d, c))
                .cmp(&ComparableLine::new(origin, geo::Line::new(a, b))),
            std::cmp::Ordering::Less
        );
        assert_ne!(
            ComparableLine::new(origin, geo::Line::new(c, d))
                .cmp(&ComparableLine::new(origin, geo::Line::new(b, a))),
            std::cmp::Ordering::Less
        );
        assert_ne!(
            ComparableLine::new(origin, geo::Line::new(d, c))
                .cmp(&ComparableLine::new(origin, geo::Line::new(b, a))),
            std::cmp::Ordering::Less
        );
    }

    fn test_line_segments_are_equal(
        origin: Point<f64>,
        a: Point<f64>,
        b: Point<f64>,
        c: Point<f64>,
        d: Point<f64>,
    ) {
        assert_ne!(
            ComparableLine::new(origin, geo::Line::new(a, b))
                .cmp(&ComparableLine::new(origin, geo::Line::new(c, d))),
            std::cmp::Ordering::Less
        );
        assert_ne!(
            ComparableLine::new(origin, geo::Line::new(b, a))
                .cmp(&ComparableLine::new(origin, geo::Line::new(c, d))),
            std::cmp::Ordering::Less
        );
        assert_ne!(
            ComparableLine::new(origin, geo::Line::new(a, b))
                .cmp(&ComparableLine::new(origin, geo::Line::new(d, c))),
            std::cmp::Ordering::Less
        );
        assert_ne!(
            ComparableLine::new(origin, geo::Line::new(b, a))
                .cmp(&ComparableLine::new(origin, geo::Line::new(d, c))),
            std::cmp::Ordering::Less
        );

        assert_ne!(
            ComparableLine::new(origin, geo::Line::new(c, d))
                .cmp(&ComparableLine::new(origin, geo::Line::new(a, b))),
            std::cmp::Ordering::Less
        );
        assert_ne!(
            ComparableLine::new(origin, geo::Line::new(d, c))
                .cmp(&ComparableLine::new(origin, geo::Line::new(a, b))),
            std::cmp::Ordering::Less
        );
        assert_ne!(
            ComparableLine::new(origin, geo::Line::new(c, d))
                .cmp(&ComparableLine::new(origin, geo::Line::new(b, a))),
            std::cmp::Ordering::Less
        );
        assert_ne!(
            ComparableLine::new(origin, geo::Line::new(d, c))
                .cmp(&ComparableLine::new(origin, geo::Line::new(b, a))),
            std::cmp::Ordering::Less
        );
    }
}
