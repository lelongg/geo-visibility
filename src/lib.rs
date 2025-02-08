//! This crate contains algorithms to compute [visibility polygon](https://www.wikiwand.com/en/Visibility_polygon).
//!
//! This code is a Rust port of the C++ lib [visibility](https://github.com/trylock/visibility).
//!
//! # Example
//!
//! The following example shows how to compute the visibility polygon of a point amongst line obstacles.  
//! The [`visibility`] method is provided by the [`Visibility`] trait which is implemented for most [geo-types](https://docs.rs/geo-types/0.4.3/geo_types/).
//!
//! ```
//! # fn main() {
//! use geo::{Coord, Line};
//! use geo_visibility::Visibility;
//!
//! let point = geo::Point::new(0.0, 0.0);
//!
//! let lines = vec![
//!     Line::new(
//!         Coord { x: 1.0, y: 1.0 },
//!         Coord { x: 1.0, y: -1.0 },
//!     ),
//!     Line::new(
//!         Coord { x: -1.0, y: -1.0 },
//!         Coord { x: -1.0, y: -2.0 },
//!     ),
//! ];
//!
//! let visibility_polygon = point.visibility(lines.as_slice());
//! # }
//! ```
//!
//! [`Visibility`]: visibility/trait.Visibility.html
//! [`visibility`]: visibility/trait.Visibility.html#method.visibility

mod angle_comparator;
mod comparable_line;
mod orientation;
mod ray;
mod utils;
mod visibility;
mod visibility_event;

pub use visibility::Visibility;
