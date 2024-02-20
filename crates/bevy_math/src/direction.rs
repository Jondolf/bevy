use crate::{
    primitives::{Primitive2d, Primitive3d},
    Quat, Vec2, Vec3, Vec3A,
};

/// An error indicating that a direction is invalid.
#[derive(Debug, PartialEq)]
pub enum InvalidDirectionError {
    /// The length of the direction vector is zero or very close to zero.
    Zero,
    /// The length of the direction vector is `std::f32::INFINITY`.
    Infinite,
    /// The length of the direction vector is `NaN`.
    NaN,
}

impl InvalidDirectionError {
    /// Creates an [`InvalidDirectionError`] from the length of an invalid direction vector.
    pub fn from_length(length: f32) -> Self {
        if length.is_nan() {
            InvalidDirectionError::NaN
        } else if !length.is_finite() {
            // If the direction is non-finite but also not NaN, it must be infinite
            InvalidDirectionError::Infinite
        } else {
            // If the direction is invalid but neither NaN nor infinite, it must be zero
            InvalidDirectionError::Zero
        }
    }
}

impl std::fmt::Display for InvalidDirectionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Direction can not be zero (or very close to zero), or non-finite."
        )
    }
}

/// A normalized vector pointing in a direction in 2D space
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct Dir2(Vec2);
impl Primitive2d for Dir2 {}

impl Dir2 {
    /// A unit vector pointing along the positive X axis.
    pub const X: Self = Self(Vec2::X);
    /// A unit vector pointing along the positive Y axis.
    pub const Y: Self = Self(Vec2::Y);
    /// A unit vector pointing along the negative X axis.
    pub const NEG_X: Self = Self(Vec2::NEG_X);
    /// A unit vector pointing along the negative Y axis.
    pub const NEG_Y: Self = Self(Vec2::NEG_Y);

    /// Create a direction from a finite, nonzero [`Vec2`].
    ///
    /// Returns [`Err(InvalidDirectionError)`](InvalidDirectionError) if the length
    /// of the given vector is zero (or very close to zero), infinite, or `NaN`.
    pub fn new(value: Vec2) -> Result<Self, InvalidDirectionError> {
        Self::new_and_length(value).map(|(dir, _)| dir)
    }

    /// Create a [`Dir2`] from a [`Vec2`] that is already normalized.
    ///
    /// # Warning
    ///
    /// `value` must be normalized, i.e it's length must be `1.0`.
    pub fn new_unchecked(value: Vec2) -> Self {
        debug_assert!(value.is_normalized());

        Self(value)
    }

    /// Create a direction from a finite, nonzero [`Vec2`], also returning its original length.
    ///
    /// Returns [`Err(InvalidDirectionError)`](InvalidDirectionError) if the length
    /// of the given vector is zero (or very close to zero), infinite, or `NaN`.
    pub fn new_and_length(value: Vec2) -> Result<(Self, f32), InvalidDirectionError> {
        let length = value.length();
        let direction = (length.is_finite() && length > 0.0).then_some(value / length);

        direction
            .map(|dir| (Self(dir), length))
            .ok_or(InvalidDirectionError::from_length(length))
    }

    /// Create a direction from its `x` and `y` components.
    ///
    /// Returns [`Err(InvalidDirectionError)`](InvalidDirectionError) if the length
    /// of the vector formed by the components is zero (or very close to zero), infinite, or `NaN`.
    pub fn from_xy(x: f32, y: f32) -> Result<Self, InvalidDirectionError> {
        Self::new(Vec2::new(x, y))
    }
}

impl TryFrom<Vec2> for Dir2 {
    type Error = InvalidDirectionError;

    fn try_from(value: Vec2) -> Result<Self, Self::Error> {
        Self::new(value)
    }
}

impl std::ops::Deref for Dir2 {
    type Target = Vec2;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl std::ops::Neg for Dir2 {
    type Output = Self;
    fn neg(self) -> Self::Output {
        Self(-self.0)
    }
}

#[cfg(feature = "approx")]
impl approx::AbsDiffEq for Dir2 {
    type Epsilon = f32;
    fn default_epsilon() -> f32 {
        f32::EPSILON
    }
    fn abs_diff_eq(&self, other: &Self, epsilon: f32) -> bool {
        self.as_ref().abs_diff_eq(other.as_ref(), epsilon)
    }
}

#[cfg(feature = "approx")]
impl approx::RelativeEq for Dir2 {
    fn default_max_relative() -> f32 {
        f32::EPSILON
    }
    fn relative_eq(&self, other: &Self, epsilon: f32, max_relative: f32) -> bool {
        self.as_ref()
            .relative_eq(other.as_ref(), epsilon, max_relative)
    }
}

#[cfg(feature = "approx")]
impl approx::UlpsEq for Dir2 {
    fn default_max_ulps() -> u32 {
        4
    }
    fn ulps_eq(&self, other: &Self, epsilon: f32, max_ulps: u32) -> bool {
        self.as_ref().ulps_eq(other.as_ref(), epsilon, max_ulps)
    }
}

/// A normalized vector pointing in a direction in 3D space
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct Dir3(Vec3);
impl Primitive3d for Dir3 {}

impl Dir3 {
    /// A unit vector pointing along the positive X axis.
    pub const X: Self = Self(Vec3::X);
    /// A unit vector pointing along the positive Y axis.
    pub const Y: Self = Self(Vec3::Y);
    /// A unit vector pointing along the positive Z axis.
    pub const Z: Self = Self(Vec3::Z);
    /// A unit vector pointing along the negative X axis.
    pub const NEG_X: Self = Self(Vec3::NEG_X);
    /// A unit vector pointing along the negative Y axis.
    pub const NEG_Y: Self = Self(Vec3::NEG_Y);
    /// A unit vector pointing along the negative Z axis.
    pub const NEG_Z: Self = Self(Vec3::NEG_Z);

    /// Create a direction from a finite, nonzero [`Vec3`].
    ///
    /// Returns [`Err(InvalidDirectionError)`](InvalidDirectionError) if the length
    /// of the given vector is zero (or very close to zero), infinite, or `NaN`.
    pub fn new(value: Vec3) -> Result<Self, InvalidDirectionError> {
        Self::new_and_length(value).map(|(dir, _)| dir)
    }

    /// Create a [`Dir3`] from a [`Vec3`] that is already normalized.
    ///
    /// # Warning
    ///
    /// `value` must be normalized, i.e it's length must be `1.0`.
    pub fn new_unchecked(value: Vec3) -> Self {
        debug_assert!(value.is_normalized());

        Self(value)
    }

    /// Create a direction from a finite, nonzero [`Vec3`], also returning its original length.
    ///
    /// Returns [`Err(InvalidDirectionError)`](InvalidDirectionError) if the length
    /// of the given vector is zero (or very close to zero), infinite, or `NaN`.
    pub fn new_and_length(value: Vec3) -> Result<(Self, f32), InvalidDirectionError> {
        let length = value.length();
        let direction = (length.is_finite() && length > 0.0).then_some(value / length);

        direction
            .map(|dir| (Self(dir), length))
            .ok_or(InvalidDirectionError::from_length(length))
    }

    /// Create a direction from its `x`, `y`, and `z` components.
    ///
    /// Returns [`Err(InvalidDirectionError)`](InvalidDirectionError) if the length
    /// of the vector formed by the components is zero (or very close to zero), infinite, or `NaN`.
    pub fn from_xyz(x: f32, y: f32, z: f32) -> Result<Self, InvalidDirectionError> {
        Self::new(Vec3::new(x, y, z))
    }
}

impl TryFrom<Vec3> for Dir3 {
    type Error = InvalidDirectionError;

    fn try_from(value: Vec3) -> Result<Self, Self::Error> {
        Self::new(value)
    }
}

impl From<Dir3> for Vec3 {
    fn from(value: Dir3) -> Self {
        value.0
    }
}

impl std::ops::Deref for Dir3 {
    type Target = Vec3;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl std::ops::Neg for Dir3 {
    type Output = Self;
    fn neg(self) -> Self::Output {
        Self(-self.0)
    }
}

impl std::ops::Mul<f32> for Dir3 {
    type Output = Vec3;
    fn mul(self, rhs: f32) -> Self::Output {
        self.0 * rhs
    }
}

impl std::ops::Mul<Dir3> for Quat {
    type Output = Dir3;

    /// Rotates the [`Dir3`] using a [`Quat`].
    fn mul(self, direction: Dir3) -> Self::Output {
        let rotated = self * *direction;

        // Make sure the result is normalized.
        // This can fail for non-unit quaternions.
        debug_assert!(rotated.is_normalized());

        Dir3::new_unchecked(rotated)
    }
}

#[cfg(feature = "approx")]
impl approx::AbsDiffEq for Dir3 {
    type Epsilon = f32;
    fn default_epsilon() -> f32 {
        f32::EPSILON
    }
    fn abs_diff_eq(&self, other: &Self, epsilon: f32) -> bool {
        self.as_ref().abs_diff_eq(other.as_ref(), epsilon)
    }
}

#[cfg(feature = "approx")]
impl approx::RelativeEq for Dir3 {
    fn default_max_relative() -> f32 {
        f32::EPSILON
    }
    fn relative_eq(&self, other: &Self, epsilon: f32, max_relative: f32) -> bool {
        self.as_ref()
            .relative_eq(other.as_ref(), epsilon, max_relative)
    }
}

#[cfg(feature = "approx")]
impl approx::UlpsEq for Dir3 {
    fn default_max_ulps() -> u32 {
        4
    }
    fn ulps_eq(&self, other: &Self, epsilon: f32, max_ulps: u32) -> bool {
        self.as_ref().ulps_eq(other.as_ref(), epsilon, max_ulps)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::InvalidDirectionError;

    #[test]
    fn dir2_creation() {
        assert_eq!(Dir2::new(Vec2::X * 12.5), Ok(Dir2::X));
        assert_eq!(
            Dir2::new(Vec2::new(0.0, 0.0)),
            Err(InvalidDirectionError::Zero)
        );
        assert_eq!(
            Dir2::new(Vec2::new(f32::INFINITY, 0.0)),
            Err(InvalidDirectionError::Infinite)
        );
        assert_eq!(
            Dir2::new(Vec2::new(f32::NEG_INFINITY, 0.0)),
            Err(InvalidDirectionError::Infinite)
        );
        assert_eq!(
            Dir2::new(Vec2::new(f32::NAN, 0.0)),
            Err(InvalidDirectionError::NaN)
        );
        assert_eq!(Dir2::new_and_length(Vec2::X * 6.5), Ok((Dir2::X, 6.5)));
    }

    #[test]
    fn dir3_creation() {
        assert_eq!(Dir3::new(Vec3::X * 12.5), Ok(Dir3::X));
        assert_eq!(
            Dir3::new(Vec3::new(0.0, 0.0, 0.0)),
            Err(InvalidDirectionError::Zero)
        );
        assert_eq!(
            Dir3::new(Vec3::new(f32::INFINITY, 0.0, 0.0)),
            Err(InvalidDirectionError::Infinite)
        );
        assert_eq!(
            Dir3::new(Vec3::new(f32::NEG_INFINITY, 0.0, 0.0)),
            Err(InvalidDirectionError::Infinite)
        );
        assert_eq!(
            Dir3::new(Vec3::new(f32::NAN, 0.0, 0.0)),
            Err(InvalidDirectionError::NaN)
        );
        assert_eq!(Dir3::new_and_length(Vec3::X * 6.5), Ok((Dir3::X, 6.5)));

        // Test rotation
        assert!(
            (Quat::from_rotation_z(std::f32::consts::FRAC_PI_2) * Dir3::X)
                .abs_diff_eq(Vec3::Y, 10e-6)
        );
    }
}
