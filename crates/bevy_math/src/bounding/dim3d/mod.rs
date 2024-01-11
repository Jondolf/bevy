mod bounded_impls;

use super::{BoundingVolume, IntersectsVolume};
use crate::prelude::{Quat, Vec3};

/// Computes the geometric center of the given set of points.
#[inline(always)]
fn point_cloud_3d_center<I>(points: I) -> Vec3
where
    I: IntoIterator<Item = Vec3> + Clone,
{
    let len = points.clone().into_iter().count();
    assert!(len >= 1, "can not compute the center of less than 1 point");

    let iter = points.into_iter();

    let denom = 1.0 / len as f32;
    iter.fold(Vec3::ZERO, |acc, point| acc + point * denom)
}

/// A trait with methods that return 3D bounded volumes for a shape
pub trait Bounded3d {
    /// Get an axis-aligned bounding box for the shape with the given translation and rotation
    fn aabb_3d(&self, translation: Vec3, rotation: Quat) -> Aabb3d;
    /// Get a bounding sphere for the shape with the given translation and rotation
    fn bounding_sphere(&self, translation: Vec3, rotation: Quat) -> BoundingSphere;
}

/// A 3D axis-aligned bounding box
#[derive(Clone, Debug)]
pub struct Aabb3d {
    /// The minimum point of the box
    pub min: Vec3,
    /// The maximum point of the box
    pub max: Vec3,
}

impl Aabb3d {
    /// Computes the smallest [`Aabb3d`] containing the given set of points,
    /// transformed by `translation` and `rotation`.
    #[inline(always)]
    pub fn from_point_cloud(
        translation: Vec3,
        rotation: Quat,
        points: impl IntoIterator<Item = Vec3>,
    ) -> Aabb3d {
        // Transform all points by rotation
        let mut iter = points.into_iter().map(|point| rotation * point);

        let first = iter
            .next()
            .expect("point cloud must contain at least one point for Aabb3d construction");

        let (min, max) = iter.fold((first, first), |(prev_min, prev_max), point| {
            (point.min(prev_min), point.max(prev_max))
        });

        Aabb3d {
            min: min + translation,
            max: max + translation,
        }
    }

    /// Computes the smallest [`BoundingSphere`] containing this [`Aabb3d`].
    #[inline(always)]
    pub fn bounding_sphere(&self) -> BoundingSphere {
        let radius = self.min.distance(self.max) / 2.0;
        BoundingSphere::new(self.center(), radius)
    }
}

impl BoundingVolume for Aabb3d {
    type Position = Vec3;
    type HalfSize = Vec3;

    #[inline(always)]
    fn center(&self) -> Self::Position {
        (self.min + self.max) / 2.
    }

    #[inline(always)]
    fn half_size(&self) -> Self::HalfSize {
        (self.max - self.min) / 2.
    }

    #[inline(always)]
    fn visible_area(&self) -> f32 {
        let b = self.max - self.min;
        b.x * (b.y + b.z) + b.y * b.z
    }

    #[inline(always)]
    fn contains(&self, other: &Self) -> bool {
        other.min.x >= self.min.x
            && other.min.y >= self.min.y
            && other.min.z >= self.min.z
            && other.max.x <= self.max.x
            && other.max.y <= self.max.y
            && other.max.z <= self.max.z
    }

    #[inline(always)]
    fn merge(&self, other: &Self) -> Self {
        Self {
            min: self.min.min(other.min),
            max: self.max.max(other.max),
        }
    }

    #[inline(always)]
    fn grow(&self, amount: Self::HalfSize) -> Self {
        let b = Self {
            min: self.min - amount,
            max: self.max + amount,
        };
        debug_assert!(b.min.x <= b.max.x && b.min.y <= b.max.y && b.min.z <= b.max.z);
        b
    }

    #[inline(always)]
    fn shrink(&self, amount: Self::HalfSize) -> Self {
        let b = Self {
            min: self.min + amount,
            max: self.max - amount,
        };
        debug_assert!(b.min.x <= b.max.x && b.min.y <= b.max.y && b.min.z <= b.max.z);
        b
    }
}

impl IntersectsVolume<Self> for Aabb3d {
    #[inline]
    fn intersects(&self, volume: &Self) -> bool {
        self.min.cmplt(volume.max).all() && self.max.cmpgt(volume.min).all()
    }
}

#[cfg(test)]
mod aabb3d_tests {
    use super::Aabb3d;
    use crate::{bounding::BoundingVolume, Vec3};

    #[test]
    fn center() {
        let aabb = Aabb3d {
            min: Vec3::new(-0.5, -1., -0.5),
            max: Vec3::new(1., 1., 2.),
        };
        assert!((aabb.center() - Vec3::new(0.25, 0., 0.75)).length() < std::f32::EPSILON);
        let aabb = Aabb3d {
            min: Vec3::new(5., 5., -10.),
            max: Vec3::new(10., 10., -5.),
        };
        assert!((aabb.center() - Vec3::new(7.5, 7.5, -7.5)).length() < std::f32::EPSILON);
    }

    #[test]
    fn half_size() {
        let aabb = Aabb3d {
            min: Vec3::new(-0.5, -1., -0.5),
            max: Vec3::new(1., 1., 2.),
        };
        assert!((aabb.half_size() - Vec3::new(0.75, 1., 1.25)).length() < std::f32::EPSILON);
    }

    #[test]
    fn area() {
        let aabb = Aabb3d {
            min: Vec3::new(-1., -1., -1.),
            max: Vec3::new(1., 1., 1.),
        };
        assert!((aabb.visible_area() - 12.).abs() < std::f32::EPSILON);
        let aabb = Aabb3d {
            min: Vec3::new(0., 0., 0.),
            max: Vec3::new(1., 0.5, 0.25),
        };
        assert!((aabb.visible_area() - 0.875).abs() < std::f32::EPSILON);
    }

    #[test]
    fn contains() {
        let a = Aabb3d {
            min: Vec3::new(-1., -1., -1.),
            max: Vec3::new(1., 1., 1.),
        };
        let b = Aabb3d {
            min: Vec3::new(-2., -1., -1.),
            max: Vec3::new(1., 1., 1.),
        };
        assert!(!a.contains(&b));
        let b = Aabb3d {
            min: Vec3::new(-0.25, -0.8, -0.9),
            max: Vec3::new(1., 1., 0.9),
        };
        assert!(a.contains(&b));
    }

    #[test]
    fn merge() {
        let a = Aabb3d {
            min: Vec3::new(-1., -1., -1.),
            max: Vec3::new(1., 0.5, 1.),
        };
        let b = Aabb3d {
            min: Vec3::new(-2., -0.5, -0.),
            max: Vec3::new(0.75, 1., 2.),
        };
        let merged = a.merge(&b);
        assert!((merged.min - Vec3::new(-2., -1., -1.)).length() < std::f32::EPSILON);
        assert!((merged.max - Vec3::new(1., 1., 2.)).length() < std::f32::EPSILON);
        assert!(merged.contains(&a));
        assert!(merged.contains(&b));
        assert!(!a.contains(&merged));
        assert!(!b.contains(&merged));
    }

    #[test]
    fn grow() {
        let a = Aabb3d {
            min: Vec3::new(-1., -1., -1.),
            max: Vec3::new(1., 1., 1.),
        };
        let padded = a.grow(Vec3::ONE);
        assert!((padded.min - Vec3::new(-2., -2., -2.)).length() < std::f32::EPSILON);
        assert!((padded.max - Vec3::new(2., 2., 2.)).length() < std::f32::EPSILON);
        assert!(padded.contains(&a));
        assert!(!a.contains(&padded));
    }

    #[test]
    fn shrink() {
        let a = Aabb3d {
            min: Vec3::new(-2., -2., -2.),
            max: Vec3::new(2., 2., 2.),
        };
        let shrunk = a.shrink(Vec3::ONE);
        assert!((shrunk.min - Vec3::new(-1., -1., -1.)).length() < std::f32::EPSILON);
        assert!((shrunk.max - Vec3::new(1., 1., 1.)).length() < std::f32::EPSILON);
        assert!(a.contains(&shrunk));
        assert!(!shrunk.contains(&a));
    }
}

use crate::primitives::Sphere;

/// A bounding sphere
#[derive(Clone, Debug)]
pub struct BoundingSphere {
    /// The center of the bounding sphere
    pub center: Vec3,
    /// The sphere
    pub sphere: Sphere,
}

impl BoundingSphere {
    /// Construct a bounding sphere from its center and radius.
    pub fn new(center: Vec3, radius: f32) -> Self {
        debug_assert!(radius >= 0.);
        Self {
            center,
            sphere: Sphere { radius },
        }
    }

    /// Computes the smallest [`BoundingSphere`] containing the given set of points,
    /// translated by `translation`.
    #[inline(always)]
    pub fn from_point_cloud<I>(translation: Vec3, rotation: Quat, points: I) -> BoundingSphere
    where
        I: IntoIterator<Item = Vec3> + Clone,
    {
        let center = point_cloud_3d_center(points.clone());
        let mut radius_squared = 0.0;

        for point in points.into_iter() {
            // Get squared version to avoid unnecessary sqrt calls
            let distance_squared = point.distance_squared(center);
            if distance_squared > radius_squared {
                radius_squared = distance_squared;
            }
        }

        BoundingSphere::new(rotation * center + translation, radius_squared.sqrt())
    }

    /// Get the radius of the bounding sphere
    #[inline(always)]
    pub fn radius(&self) -> f32 {
        self.sphere.radius
    }
}

impl BoundingVolume for BoundingSphere {
    type Position = Vec3;
    type HalfSize = f32;

    #[inline(always)]
    fn center(&self) -> Self::Position {
        self.center
    }

    #[inline(always)]
    fn half_size(&self) -> Self::HalfSize {
        self.radius()
    }

    #[inline(always)]
    fn visible_area(&self) -> f32 {
        2. * std::f32::consts::PI * self.radius() * self.radius()
    }

    #[inline(always)]
    fn contains(&self, other: &Self) -> bool {
        let diff = self.radius() - other.radius();
        self.center.distance_squared(other.center) <= diff.powi(2).copysign(diff)
    }

    #[inline(always)]
    fn merge(&self, other: &Self) -> Self {
        let diff = other.center - self.center;
        let length = diff.length();
        if self.radius() >= length + other.radius() {
            return self.clone();
        }
        if other.radius() >= length + self.radius() {
            return other.clone();
        }
        let dir = diff / length;
        Self::new(
            (self.center + other.center) / 2. + dir * ((other.radius() - self.radius()) / 2.),
            (length + self.radius() + other.radius()) / 2.,
        )
    }

    #[inline(always)]
    fn grow(&self, amount: Self::HalfSize) -> Self {
        debug_assert!(amount >= 0.);
        Self {
            center: self.center,
            sphere: Sphere {
                radius: self.radius() + amount,
            },
        }
    }

    #[inline(always)]
    fn shrink(&self, amount: Self::HalfSize) -> Self {
        debug_assert!(amount >= 0.);
        debug_assert!(self.radius() >= amount);
        Self {
            center: self.center,
            sphere: Sphere {
                radius: self.radius() - amount,
            },
        }
    }
}

impl IntersectsVolume<Self> for BoundingSphere {
    #[inline]
    fn intersects(&self, volume: &Self) -> bool {
        let delta_center = volume.center - self.center;
        let distance_squared = delta_center.length_squared();
        distance_squared <= self.radius() + volume.radius()
    }
}

#[cfg(test)]
mod bounding_sphere_tests {
    use super::BoundingSphere;
    use crate::{bounding::BoundingVolume, Vec3};

    #[test]
    fn area() {
        let sphere = BoundingSphere::new(Vec3::ONE, 5.);
        // Since this number is messy we check it with a higher threshold
        assert!((sphere.visible_area() - 157.0796).abs() < 0.001);
    }

    #[test]
    fn contains() {
        let a = BoundingSphere::new(Vec3::ONE, 5.);
        let b = BoundingSphere::new(Vec3::new(5.5, 1., 1.), 1.);
        assert!(!a.contains(&b));
        let b = BoundingSphere::new(Vec3::new(1., -3.5, 1.), 0.5);
        assert!(a.contains(&b));
    }

    #[test]
    fn contains_identical() {
        let a = BoundingSphere::new(Vec3::ONE, 5.);
        assert!(a.contains(&a));
    }

    #[test]
    fn merge() {
        // When merging two circles that don't contain each other, we find a center position that
        // contains both
        let a = BoundingSphere::new(Vec3::ONE, 5.);
        let b = BoundingSphere::new(Vec3::new(1., 1., -4.), 1.);
        let merged = a.merge(&b);
        assert!((merged.center - Vec3::new(1., 1., 0.5)).length() < std::f32::EPSILON);
        assert!((merged.radius() - 5.5).abs() < std::f32::EPSILON);
        assert!(merged.contains(&a));
        assert!(merged.contains(&b));
        assert!(!a.contains(&merged));
        assert!(!b.contains(&merged));

        // When one circle contains the other circle, we use the bigger circle
        let b = BoundingSphere::new(Vec3::ZERO, 3.);
        assert!(a.contains(&b));
        let merged = a.merge(&b);
        assert_eq!(merged.center, a.center);
        assert_eq!(merged.radius(), a.radius());

        // When two circles are at the same point, we use the bigger radius
        let b = BoundingSphere::new(Vec3::ONE, 6.);
        let merged = a.merge(&b);
        assert_eq!(merged.center, a.center);
        assert_eq!(merged.radius(), b.radius());
    }

    #[test]
    fn merge_identical() {
        let a = BoundingSphere::new(Vec3::ONE, 5.);
        let merged = a.merge(&a);
        assert_eq!(merged.center, a.center);
        assert_eq!(merged.radius(), a.radius());
    }

    #[test]
    fn grow() {
        let a = BoundingSphere::new(Vec3::ONE, 5.);
        let padded = a.grow(1.25);
        assert!((padded.radius() - 6.25).abs() < std::f32::EPSILON);
        assert!(padded.contains(&a));
        assert!(!a.contains(&padded));
    }

    #[test]
    fn shrink() {
        let a = BoundingSphere::new(Vec3::ONE, 5.);
        let shrunk = a.shrink(0.5);
        assert!((shrunk.radius() - 4.5).abs() < std::f32::EPSILON);
        assert!(a.contains(&shrunk));
        assert!(!shrunk.contains(&a));
    }
}
