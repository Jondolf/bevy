#![cfg_attr(docsrs, feature(doc_auto_cfg))]
#![forbid(unsafe_code)]
#![doc(
    html_logo_url = "https://bevyengine.org/assets/icon.png",
    html_favicon_url = "https://bevyengine.org/assets/icon.png"
)]

//! Provides math types and functionality for the Bevy game engine.
//!
//! The commonly used types are vectors like [`Vec2`] and [`Vec3`],
//! matrices like [`Mat2`], [`Mat3`] and [`Mat4`] and orientation representations
//! like [`Quat`].

mod affine3;
mod aspect_ratio;
pub mod bounding;
pub mod common_traits;
mod compass;
pub mod cubic_splines;
pub mod curve;
mod direction;
mod float_ord;
mod isometry;
pub mod ops;
pub mod primitives;
mod ray;
pub mod ray_cast;
mod rects;
mod rotation2d;
#[cfg(feature = "rand")]
pub mod sampling;
pub use compass::{CompassOctant, CompassQuadrant};

pub use affine3::*;
pub use aspect_ratio::AspectRatio;
pub use common_traits::*;
pub use direction::*;
pub use float_ord::*;
pub use isometry::{Isometry2d, Isometry3d};
pub use ops::FloatPow;
pub use ray::{Ray2d, Ray3d};
pub use rects::*;
pub use rotation2d::Rot2;
#[cfg(feature = "rand")]
pub use sampling::FromRng;
#[cfg(feature = "rand")]
pub use sampling::ShapeSample;

/// The math prelude.
///
/// This includes the most common types in this crate, re-exported for your convenience.
pub mod prelude {
    #[doc(hidden)]
    pub use crate::{
        cubic_splines::{
            CubicBSpline, CubicBezier, CubicCardinalSpline, CubicCurve, CubicGenerator,
            CubicHermite, CubicNurbs, CubicNurbsError, CubicSegment, CyclicCubicGenerator,
            RationalCurve, RationalGenerator, RationalSegment,
        },
        curve::*,
        direction::{Dir2, Dir3, Dir3A},
        ops,
        primitives::*,
        ray_cast::{RayCast2d, RayCast3d, RayHit2d, RayHit3d},
        BVec2, BVec3, BVec4, EulerRot, FloatExt, IRect, IVec2, IVec3, IVec4, Isometry2d,
        Isometry3d, Mat2, Mat3, Mat4, Quat, Ray2d, Ray3d, Rect, Rot2, StableInterpolate, URect,
        UVec2, UVec3, UVec4, Vec2, Vec2Swizzles, Vec3, Vec3Swizzles, Vec4, Vec4Swizzles,
    };

    #[doc(hidden)]
    #[cfg(feature = "rand")]
    pub use crate::sampling::{FromRng, ShapeSample};
}

pub use glam::*;
