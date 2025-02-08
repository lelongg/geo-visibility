use crate::angle_comparator::AngleComparator;
use crate::comparable_line::ComparableLine;
use crate::orientation::Orientation;
use crate::ray::Ray;
use crate::utils::approx_equal;
use crate::visibility_event::{VisibilityEvent, VisibilityEventType};
use approx::*;
use geo_clipper::Clipper;
use log::warn;
use std::collections::BTreeSet;

pub trait Visibility<T: ?Sized> {
    fn visibility(&self, obstacles: &T) -> geo::Polygon<f64>;
}

impl Visibility<geo::Polygon<f64>> for geo::Point<f64> {
    fn visibility(&self, obstacles: &geo::Polygon<f64>) -> geo::Polygon<f64> {
        let segments: Vec<_> = obstacles
            .exterior()
            .lines()
            .chain(
                obstacles
                    .interiors()
                    .iter()
                    .flat_map(|interior| interior.lines()),
            )
            .collect();
        self.visibility(segments.as_slice())
    }
}

impl Visibility<geo::MultiPolygon<f64>> for geo::Point<f64> {
    fn visibility(&self, obstacles: &geo::MultiPolygon<f64>) -> geo::Polygon<f64> {
        let segments: Vec<_> = obstacles
            .0
            .iter()
            .flat_map(|obstacle| {
                obstacle
                    .exterior()
                    .lines()
                    .chain(obstacles.0.iter().flat_map(|obstacle| {
                        obstacle
                            .interiors()
                            .iter()
                            .flat_map(|interior| interior.lines())
                    }))
            })
            .collect();
        self.visibility(segments.as_slice())
    }
}

impl Visibility<[geo::Line<f64>]> for geo::Point<f64> {
    fn visibility(&self, obstacles: &[geo::Line<f64>]) -> geo::Polygon<f64> {
        let mut state = BTreeSet::new();
        let mut events = Vec::with_capacity(obstacles.len() * 2 + 1);

        for segment in obstacles {
            let a = geo::Point::from(segment.start);
            let b = geo::Point::from(segment.end);

            // Sort line segment endpoints and add them as events
            // Skip line segments collinear with the point
            match Orientation::from(*self, a, b) {
                Orientation::Collinear => {
                    continue;
                }
                Orientation::RightTurn => {
                    events.push(VisibilityEvent::start(segment));
                    events.push(VisibilityEvent::end(&geo::Line::new(
                        segment.end,
                        segment.start,
                    )));
                }
                Orientation::LeftTurn => {
                    events.push(VisibilityEvent::start(&geo::Line::new(
                        segment.end,
                        segment.start,
                    )));
                    events.push(VisibilityEvent::end(segment));
                }
            }

            // Initialize state by adding line segments that are intersected by vertical ray from the point
            let (a, b) = if a.x() > b.x() { (b, a) } else { (a, b) };
            let abp = Orientation::from(a, b, *self);

            if (abs_diff_eq!(b.x(), self.x()) || (a.x() < self.x() && self.x() < b.x()))
                && abp == Orientation::RightTurn
            {
                state.insert(ComparableLine::new(*self, *segment));
            }
        }

        // sort events by angle
        sort_events_by_angle(self, &mut events);

        // find the visibility polygon
        let mut vertices = Vec::new();
        for event in events {
            let segment = ComparableLine::new(*self, event.segment);

            if event.event_type == VisibilityEventType::EndVertex {
                state.remove(&segment);
            }

            if let Some(first_state) = state.iter().next() {
                if segment < *first_state {
                    let ray = Ray::new(geo::Line::new(*self, event.point()));
                    if let Some(intersection) = ray.intersects(&first_state.line) {
                        match event.event_type {
                            VisibilityEventType::StartVertex => {
                                vertices.push(intersection);
                                vertices.push(event.point());
                            }
                            VisibilityEventType::EndVertex => {
                                vertices.push(event.point());
                                vertices.push(intersection);
                            }
                        }
                    } else {
                        warn!("ray intersects a line segment iff the line segment is in the state");
                    }
                }
            } else {
                vertices.push(event.point());
            }

            if event.event_type == VisibilityEventType::StartVertex {
                state.insert(segment);
            }
        }

        // remove collinear points
        let mut top = 0;
        for it in 0..vertices.len() {
            let prev = if top == 0 {
                vertices.len() - 1
            } else {
                top - 1
            };
            let next = if it + 1 == vertices.len() { 0 } else { it + 1 };

            if Orientation::from(vertices[prev], vertices[it], vertices[next])
                != Orientation::Collinear
            {
                vertices[top] = vertices[it];
                top += 1;
            }
        }
        vertices.truncate(top);

        geo::Polygon::new(vertices.into_iter().collect(), Vec::new())
    }
}

fn sort_events_by_angle(origin: &geo::Point<f64>, events: &mut [VisibilityEvent]) {
    let angle_comparator = AngleComparator { origin: *origin };
    events.sort_by(|a, b| {
        if approx_equal(&a.point(), &b.point()) {
            match (&a.event_type, &b.event_type) {
                (VisibilityEventType::EndVertex, VisibilityEventType::StartVertex) => {
                    std::cmp::Ordering::Less
                }
                (VisibilityEventType::StartVertex, VisibilityEventType::EndVertex) => {
                    std::cmp::Ordering::Greater
                }
                _ => std::cmp::Ordering::Greater,
            }
        } else {
            angle_comparator.cmp(&a.point(), &b.point())
        }
    });
}

/// Warning: this is not the real polygon visibility but the union of its vertices visibility
impl<T> Visibility<T> for geo::Polygon<f64>
where
    geo::Point<f64>: Visibility<T>,
{
    fn visibility(&self, obstacles: &T) -> geo::Polygon<f64> {
        let mut visibility_polygon = geo::MultiPolygon(Vec::new());
        for point in self.exterior().points().skip(1) {
            let polygon = point.visibility(obstacles);
            visibility_polygon = visibility_polygon.union(&polygon, 1000.0);
        }
        visibility_polygon
            .0
            .first()
            .cloned()
            .unwrap_or_else(|| geo::Polygon::new(geo::LineString(vec![]), vec![]))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use data_uri_utils::svg_str_to_data_uri;
    use geo::{Coord, Line};
    use geo_rand::{GeoRand, GeoRandParameters};
    use geo_svg::{Color, ToSvg};

    fn test_visibility(
        [origin_x, origin_y]: [f64; 2],
        segments: &[[[f64; 2]; 2]],
        visibility: &[[f64; 2]],
    ) {
        let origin = geo::Point::new(origin_x, origin_y);
        let lines: Vec<_> = segments
            .iter()
            .map(|[[x1, y1], [x2, y2]]| {
                geo::Line::new(geo::Coord { x: *x1, y: *y1 }, geo::Coord { x: *x2, y: *y2 })
            })
            .collect();

        let result = origin.visibility(lines.as_slice());

        println!(
            "{}",
            svg_str_to_data_uri(
                result
                    .to_svg()
                    .and(lines.to_svg())
                    .and(origin.to_svg())
                    .to_string()
            )
        );

        assert_eq!(
            result.exterior().points().count().saturating_sub(1),
            visibility.len()
        );

        for (i, point) in result
            .exterior()
            .points()
            .take(visibility.len())
            .enumerate()
        {
            assert!(approx_equal(
                &point,
                &geo::Point::new(visibility[i][0], visibility[i][1])
            ));
        }
    }

    #[test]
    fn calculate_visibility_polygon_with_no_line_segments() {
        test_visibility([0.0, 0.0], &[], &[]);
    }

    #[test]
    fn calculate_visibility_polygon_with_no_obstaces_apart_from_the_boundary() {
        test_visibility(
            [0.0, 0.0],
            &[
                [[-250.0, -250.0], [-250.0, 250.0]],
                [[-250.0, 250.0], [250.0, 250.0]],
                [[250.0, 250.0], [250.0, -250.0]],
                [[250.0, -250.0], [-250.0, -250.0]],
            ],
            &[
                [250.0, 250.0],
                [250.0, -250.0],
                [-250.0, -250.0],
                [-250.0, 250.0],
            ],
        );
    }

    #[test]
    fn calculate_visibility_polygon_with_a_polyline_as_an_obstacle() {
        test_visibility(
            [0.0, 0.0],
            &[
                [[-250.0, -250.0], [-250.0, 250.0]],
                [[-250.0, 250.0], [250.0, 250.0]],
                [[250.0, 250.0], [250.0, -250.0]],
                [[250.0, -250.0], [-250.0, -250.0]],
                [[-50.0, 50.0], [50.0, 50.0]],
                [[50.0, 50.0], [50.0, -50.0]],
            ],
            &[
                [50.0, 50.0],
                [50.0, -50.0],
                [250.0, -250.0],
                [-250.0, -250.0],
                [-250.0, 250.0],
                [-50.0, 50.0],
            ],
        );
    }

    #[test]
    fn calculate_visibility_polygon_with_a_convex_polygon_as_an_obstacle() {
        test_visibility(
            [0.0, 0.0],
            &[
                [[-250.0, -250.0], [-250.0, 250.0]],
                [[-250.0, 250.0], [250.0, 250.0]],
                [[250.0, 250.0], [250.0, -250.0]],
                [[250.0, -250.0], [-250.0, -250.0]],
                [[-50.0, 50.0], [50.0, 50.0]],
                [[50.0, 50.0], [50.0, 100.0]],
                [[50.0, 100.0], [-50.0, 100.0]],
                [[-50.0, 100.0], [-50.0, 50.0]],
            ],
            &[
                [50.0, 50.0],
                [250.0, 250.0],
                [250.0, -250.0],
                [-250.0, -250.0],
                [-250.0, 250.0],
                [-50.0, 50.0],
            ],
        );
    }

    #[test]
    fn calculate_visibility_polygon_with_a_concave_polygon_as_an_obstacle() {
        test_visibility(
            [0.0, 0.0],
            &[
                [[-250.0, -250.0], [-250.0, 250.0]],
                [[-250.0, 250.0], [250.0, 250.0]],
                [[250.0, 250.0], [250.0, -250.0]],
                [[250.0, -250.0], [-250.0, -250.0]],
                [[-50.0, 50.0], [0.0, 100.0]],
                [[0.0, 100.0], [50.0, 50.0]],
                [[50.0, 50.0], [0.0, 200.0]],
                [[0.0, 200.0], [-50.0, 50.0]],
            ],
            &[
                [0.0, 100.0],
                [50.0, 50.0],
                [250.0, 250.0],
                [250.0, -250.0],
                [-250.0, -250.0],
                [-250.0, 250.0],
                [-50.0, 50.0],
            ],
        );
    }

    #[test]
    fn calculate_visibility_polygon_with_two_polygons_as_obstacles() {
        test_visibility(
            [0.0, 0.0],
            &[
                [[-250.0, -250.0], [-250.0, 250.0]],
                [[-250.0, 250.0], [250.0, 250.0]],
                [[250.0, 250.0], [250.0, -250.0]],
                [[250.0, -250.0], [-250.0, -250.0]],
                [[-50.0, -50.0], [0.0, -100.0]],
                [[0.0, -100.0], [50.0, -50.0]],
                [[50.0, -50.0], [0.0, -200.0]],
                [[0.0, -200.0], [-50.0, -50.0]],
                [[-50.0, 50.0], [0.0, 100.0]],
                [[0.0, 100.0], [50.0, 50.0]],
                [[50.0, 50.0], [0.0, 200.0]],
                [[0.0, 200.0], [-50.0, 50.0]],
            ],
            &[
                [0.0, 100.0],
                [50.0, 50.0],
                [250.0, 250.0],
                [250.0, -250.0],
                [50.0, -50.0],
                [0.0, -100.0],
                [-50.0, -50.0],
                [-250.0, -250.0],
                [-250.0, 250.0],
                [-50.0, 50.0],
            ],
        );
    }

    #[test]
    fn show_point_visibility() {
        use rand_core::SeedableRng;
        let mut rng = rand_pcg::Pcg64::seed_from_u64(2);
        let rect = geo::Rect::new(
            geo::Coord { x: 0., y: 0.0 },
            geo::Coord { x: 400.0, y: 400.0 },
        );
        let holes = geo::MultiPolygon::rand(
            &mut rng,
            &GeoRandParameters {
                max_x: rect.width(),
                max_y: rect.height(),
                ..GeoRandParameters::default()
            },
        );
        let polygons = dbg!(geo::Polygon::from(rect).difference(&holes, 1000.0));
        println!("{}", svg_str_to_data_uri(polygons.to_svg().to_string(),));
        let polygon = polygons.0.first().unwrap().clone();
        let point = geo::Point::new(rect.width() / 2.0, rect.height() / 2.0);
        let visibility_polygon = point.visibility(&polygon);

        println!(
            "{}",
            svg_str_to_data_uri(
                polygon
                    .to_svg()
                    .and(visibility_polygon.to_svg())
                    .and(point.to_svg())
                    .to_string(),
            )
        );
    }

    #[test]
    fn show_polygon_vertices_visibility() {
        use rand_core::SeedableRng;
        let mut rng = rand_pcg::Pcg64::seed_from_u64(4);
        let rect = geo::Rect::new(
            geo::Coord { x: 0., y: 0.0 },
            geo::Coord { x: 400.0, y: 400.0 },
        );
        let rand_parameters = GeoRandParameters {
            max_x: rect.width(),
            max_y: rect.height(),
            max_polygon_vertices_count: 20,
            ..GeoRandParameters::default()
        };
        let mut holes = geo::MultiPolygon::rand(&mut rng, &rand_parameters);
        let polygons = &holes.0.drain(0..2).collect::<Vec<_>>();
        let polygon1 = &polygons[0];
        let polygon2 = &polygons[1];
        let obstacles = geo::Polygon::from(rect).difference(&holes, 1000.0).0[0].clone();
        let visibility_polygon1 = polygon1.visibility(&obstacles.difference(polygon2, 1000.0).0[0]);
        let visibility_polygon2 = polygon2.visibility(&obstacles.difference(polygon1, 1000.0).0[0]);
        let _result = visibility_polygon1
            .intersection(&visibility_polygon2, 1000.0)
            .difference(polygon1, 1000.0)
            .difference(polygon2, 1000.0);

        println!(
            "{}",
            svg_str_to_data_uri(
                obstacles
                    .difference(polygon1, 1000.0)
                    .difference(polygon2, 1000.0)
                    .to_svg()
                    .and(visibility_polygon2.to_svg())
                    .and(polygon1.to_svg())
                    .and(polygon2.to_svg())
                    .to_string(),
            )
        );
    }

    #[test]
    fn test_sort_events_by_angle() {
        let mut events = vec![
            VisibilityEvent::end(&geo::Line::new(
                geo::Coord {
                    x: 192.473_727_120_354_4,
                    y: 390.035_278_687_596_2,
                },
                geo::Coord {
                    x: 70.429_624_976_228_72,
                    y: 389.090_943_707_312_93,
                },
            )),
            VisibilityEvent::end(&geo::Line::new(
                geo::Coord {
                    x: 192.473_727_120_354_4,
                    y: 390.035_278_687_596_2,
                },
                geo::Coord {
                    x: 23.005_021_972_253_01,
                    y: 392.349_204_790_687_85,
                },
            )),
        ];

        let result = events.clone();

        sort_events_by_angle(
            &geo::Point::new(29.893_574_281_807_478, 268.926_803_395_459_73),
            &mut events,
        );

        assert_eq!(events, result);
    }

    #[test]
    fn issue_27() {
        let point = geo::Point::new(1.53, -0.4);
        let lines = vec![
            Line {
                start: Coord { x: 0.0, y: 0.5 },
                end: Coord { x: 1.5, y: 0.5 },
            },
            Line {
                start: Coord { x: 0.0, y: 0.0 },
                end: Coord { x: 0.0, y: 1.0 },
            },
            Line {
                start: Coord { x: 1.5, y: 0.5 },
                end: Coord { x: 1.5, y: 1.5 },
            },
            Line {
                start: Coord { x: 0.5, y: -0.5 },
                end: Coord { x: 0.5, y: 0.5 },
            },
            Line {
                start: Coord { x: -25.0, y: -25.0 },
                end: Coord { x: 25.0, y: -25.0 },
            },
            Line {
                start: Coord { x: 25.0, y: -25.0 },
                end: Coord { x: 25.0, y: 25.0 },
            },
            Line {
                start: Coord { x: 25.0, y: 25.0 },
                end: Coord { x: -25.0, y: 25.0 },
            },
            Line {
                start: Coord { x: -25.0, y: 25.0 },
                end: Coord { x: -25.0, y: -25.0 },
            },
        ];
        let vis = point.visibility(lines.as_slice());
        println!(
            "{}",
            svg_str_to_data_uri(
                vis.to_svg()
                    .with_color(Color::Named("green"))
                    .with_stroke_width(0.0)
                    .and(
                        lines
                            .to_svg()
                            .with_color(Color::Named("red"))
                            .with_stroke_width(0.1)
                    )
                    .and(
                        point
                            .to_svg()
                            .with_color(Color::Named("blue"))
                            .with_radius(0.1)
                            .with_stroke_width(0.0)
                    )
                    .and(
                        Coord { x: 0.0, y: 1.0 }
                            .to_svg()
                            .with_color(Color::Named("orange"))
                            .with_radius(0.1)
                            .with_stroke_width(0.0)
                    )
                    .to_string(),
            )
        );
    }
}
