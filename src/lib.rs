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
//! use geo::{Coordinate, Line};
//! use geo_visibility::Visibility;
//!
//! let lines = [
//!     Line::new(
//!         Coordinate { x: 0.0, y: 0.0 },
//!         Coordinate { x: 1.0, y: 8.0 },
//!     ),
//! ];
//!
//! let visibility_polygon = point.visibility(&lines);
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
