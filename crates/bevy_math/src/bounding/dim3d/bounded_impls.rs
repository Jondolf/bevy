use glam::{Mat3, Quat, Vec3};

use crate::primitives::{
    BoxedPolyline3d, Capsule, Cone, ConicalFrustum, Cuboid, Cylinder, Direction3d, Line3d, Plane3d,
    Polyline3d, Segment3d, Sphere, Torus,
};

use super::{Aabb3d, Bounded3d, BoundingSphere};

impl Bounded3d for Sphere {
    fn aabb_3d(&self, translation: Vec3, _rotation: Quat) -> Aabb3d {
        Aabb3d {
            min: translation - Vec3::splat(self.radius),
            max: translation + Vec3::splat(self.radius),
        }
    }

    fn bounding_sphere(&self, translation: Vec3, _rotation: Quat) -> BoundingSphere {
        BoundingSphere::new(translation, self.radius)
    }
}

impl Bounded3d for Plane3d {
    fn aabb_3d(&self, translation: Vec3, rotation: Quat) -> Aabb3d {
        // Get a direction along the plane rotated by `rotation`
        let direction = (rotation * *self.normal).any_orthonormal_vector();
        let facing_x = direction == Vec3::X || direction == Vec3::NEG_X;
        let facing_y = direction == Vec3::Y || direction == Vec3::NEG_Y;
        let facing_z = direction == Vec3::Z || direction == Vec3::NEG_Z;

        // Dividing `f32::MAX` by 2.0 can actually be good so that we can do operations
        // like growing or shrinking the AABB without breaking things.
        let max = f32::MAX / 2.0;
        let half_width = if facing_y && facing_z { 0.0 } else { max };
        let half_height = if facing_x && facing_z { 0.0 } else { max };
        let half_depth = if facing_x && facing_y { 0.0 } else { max };
        let half_size = Vec3::new(half_width, half_height, half_depth);

        Aabb3d {
            min: translation - half_size,
            max: translation + half_size,
        }
    }

    fn bounding_sphere(&self, translation: Vec3, _rotation: Quat) -> BoundingSphere {
        BoundingSphere::new(translation, f32::MAX / 2.0)
    }
}

impl Bounded3d for Line3d {
    fn aabb_3d(&self, translation: Vec3, rotation: Quat) -> Aabb3d {
        // Get the line direction along the plane rotated by `rotation`
        let direction = rotation * *self.direction;
        let x_parallel = direction == Vec3::X || direction == Vec3::NEG_X;
        let y_parallel = direction == Vec3::Y || direction == Vec3::NEG_Y;
        let z_parallel = direction == Vec3::Z || direction == Vec3::NEG_Z;

        // Dividing `f32::MAX` by 2.0 can actually be good so that we can do operations
        // like growing or shrinking the AABB without breaking things.
        let max = f32::MAX / 2.0;
        let half_width = if y_parallel && z_parallel { 0.0 } else { max };
        let half_height = if x_parallel && z_parallel { 0.0 } else { max };
        let half_depth = if x_parallel && y_parallel { 0.0 } else { max };
        let half_size = Vec3::new(half_width, half_height, half_depth);

        Aabb3d {
            min: translation - half_size,
            max: translation + half_size,
        }
    }

    fn bounding_sphere(&self, translation: Vec3, _rotation: Quat) -> BoundingSphere {
        BoundingSphere::new(translation, f32::MAX / 2.0)
    }
}

impl Bounded3d for Segment3d {
    fn aabb_3d(&self, translation: Vec3, rotation: Quat) -> Aabb3d {
        // Rotate the segment by `rotation`
        let direction = Direction3d::from_normalized(rotation * *self.direction);
        let segment = Self { direction, ..*self };
        let (point1, point2) = (segment.point1(), segment.point2());

        Aabb3d {
            min: translation + point1.min(point2),
            max: translation + point1.max(point2),
        }
    }

    fn bounding_sphere(&self, translation: Vec3, _rotation: Quat) -> BoundingSphere {
        BoundingSphere::new(translation, self.half_length)
    }
}

impl<const N: usize> Bounded3d for Polyline3d<N> {
    fn aabb_3d(&self, translation: Vec3, rotation: Quat) -> Aabb3d {
        Aabb3d::from_point_cloud(translation, rotation, self.vertices)
    }

    fn bounding_sphere(&self, translation: Vec3, rotation: Quat) -> BoundingSphere {
        BoundingSphere::from_point_cloud(translation, rotation, self.vertices)
    }
}

impl Bounded3d for BoxedPolyline3d {
    fn aabb_3d(&self, translation: Vec3, rotation: Quat) -> Aabb3d {
        Aabb3d::from_point_cloud(translation, rotation, self.vertices.to_vec())
    }

    fn bounding_sphere(&self, translation: Vec3, rotation: Quat) -> BoundingSphere {
        BoundingSphere::from_point_cloud(translation, rotation, self.vertices.to_vec())
    }
}

impl Bounded3d for Cuboid {
    fn aabb_3d(&self, translation: Vec3, rotation: Quat) -> Aabb3d {
        let abs_rot_mat = Mat3::from_cols_array(
            &Mat3::from_quat(rotation)
                .to_cols_array()
                .map(|col| col.abs()),
        );
        let half_extents = abs_rot_mat * self.half_extents;

        Aabb3d {
            min: translation - half_extents,
            max: translation + half_extents,
        }
    }

    fn bounding_sphere(&self, translation: Vec3, _rotation: Quat) -> BoundingSphere {
        BoundingSphere {
            center: translation,
            sphere: Sphere {
                radius: self.half_extents.length(),
            },
        }
    }
}

impl Bounded3d for Cylinder {
    fn aabb_3d(&self, translation: Vec3, rotation: Quat) -> Aabb3d {
        // Get segment from bottom to top
        let segment = rotation * Vec3::Y * 2.0 * self.half_height;

        // Compute half extents of AABB
        let length_squared = segment.length_squared();
        let half_extents = self.radius
            * Vec3::new(
                (segment.y.powi(2) + segment.z.powi(2)) / length_squared,
                (segment.x.powi(2) + segment.z.powi(2)) / length_squared,
                (segment.x.powi(2) + segment.y.powi(2)) / length_squared,
            );

        Aabb3d {
            min: translation - half_extents,
            max: translation + half_extents,
        }
    }

    fn bounding_sphere(&self, translation: Vec3, _rotation: Quat) -> BoundingSphere {
        let radius = (self.radius.powi(2) + self.half_height.powi(2)).sqrt();
        BoundingSphere::new(translation, radius)
    }
}

impl Bounded3d for Capsule {
    fn aabb_3d(&self, translation: Vec3, rotation: Quat) -> Aabb3d {
        // Get the line segment between the hemispheres of the rotated capsule
        let segment = Segment3d {
            direction: Direction3d::from_normalized(rotation * Vec3::Y),
            half_length: self.half_length,
        };
        let (a, b) = (segment.point1(), segment.point2());

        let min = a.min(b) - Vec3::splat(self.radius);
        let max = a.max(b) + Vec3::splat(self.radius);

        Aabb3d {
            min: min + translation,
            max: max + translation,
        }
    }

    fn bounding_sphere(&self, translation: Vec3, _rotation: Quat) -> BoundingSphere {
        BoundingSphere::new(translation, self.radius + self.half_length)
    }
}

impl Bounded3d for Cone {
    fn aabb_3d(&self, translation: Vec3, rotation: Quat) -> Aabb3d {
        let mut min = Vec3::ZERO;
        let mut max = Vec3::ZERO;
        let mut basis = Vec3::ZERO;

        for dim in 0..3 {
            basis[dim] = 1.0;
            max[dim] = cone_local_support_point(*self, basis)[dim];

            basis[dim] = -1.0;
            min[dim] = cone_local_support_point(*self, basis)[dim];

            basis[dim] = 0.0;
        }

        Aabb3d {
            min: rotation * min + translation,
            max: rotation * max + translation,
        }
    }

    fn bounding_sphere(&self, translation: Vec3, _rotation: Quat) -> BoundingSphere {
        let radius = (self.radius.powi(2) + (self.height / 2.0).powi(2)).sqrt();
        BoundingSphere::new(translation, radius)
    }
}

/// Computes the local support point of a [`Cone`]. This corresponds
/// to the farthest point on the cone in the given `direction`.
fn cone_local_support_point(cone: Cone, direction: Vec3) -> Vec3 {
    let mut support = direction;
    let half_height = cone.height / 2.0;

    support.y = 0.0;
    support = support.normalize();

    if support == Vec3::ZERO || !support.is_finite() {
        support = Vec3::ZERO;
        support.y = half_height.copysign(direction.y);
    } else {
        support *= cone.radius;
        support.y = -half_height;

        if direction.dot(support) < direction.y * half_height {
            support = Vec3::ZERO;
            support.y = half_height;
        }
    }

    support
}

impl Bounded3d for ConicalFrustum {
    fn aabb_3d(&self, translation: Vec3, rotation: Quat) -> Aabb3d {
        let mut min = Vec3::ZERO;
        let mut max = Vec3::ZERO;
        let mut basis = Vec3::ZERO;

        for dim in 0..3 {
            basis[dim] = 1.0;
            max[dim] = conical_frustum_local_support_point(*self, basis)[dim];

            basis[dim] = -1.0;
            min[dim] = conical_frustum_local_support_point(*self, basis)[dim];

            basis[dim] = 0.0;
        }

        Aabb3d {
            min: rotation * min + translation,
            max: rotation * max + translation,
        }
    }

    fn bounding_sphere(&self, translation: Vec3, _rotation: Quat) -> BoundingSphere {
        let max_base_radius = self.radius_top.max(self.radius_bottom);
        let radius = ((max_base_radius).powi(2) + (self.height / 2.0).powi(2)).sqrt();
        BoundingSphere::new(translation, radius)
    }
}

/// Computes the local support point of a [`ConicalFrustum`]. This corresponds
/// to the farthest point on the frustum in the given `direction`.
fn conical_frustum_local_support_point(frustum: ConicalFrustum, direction: Vec3) -> Vec3 {
    let mut support = direction;
    let half_height = frustum.height / 2.0;

    support.y = 0.0;
    support = support.normalize();

    if support == Vec3::ZERO || !support.is_finite() {
        support = Vec3::ZERO;
        support.y = half_height.copysign(direction.y);
    } else {
        support *= frustum.radius_bottom;
        support.y = -half_height;

        if direction.dot(support) < direction.y * half_height {
            let cylinder = Cylinder {
                radius: frustum.radius_top,
                half_height,
            };
            support = cylinder_local_support_point(cylinder, direction);
            support.y = half_height;
        }
    }

    support
}

/// Computes the local support point of a [`Cylinder`]. This corresponds
/// to the farthest point on the cylinder in the given `direction`.
fn cylinder_local_support_point(cylinder: Cylinder, direction: Vec3) -> Vec3 {
    let mut support = direction;

    support.y = 0.0;
    support = support.normalize();

    if support != Vec3::ZERO {
        support *= cylinder.radius;
    }

    support.y = cylinder.half_height.copysign(direction.y);

    support
}

impl Bounded3d for Torus {
    fn aabb_3d(&self, translation: Vec3, rotation: Quat) -> Aabb3d {
        let outer_radius = self.outer_radius();
        let half_extents = rotation * Vec3::new(outer_radius, self.minor_radius, outer_radius);
        Aabb3d {
            min: translation - half_extents,
            max: translation + half_extents,
        }
    }

    fn bounding_sphere(&self, translation: Vec3, _rotation: Quat) -> BoundingSphere {
        BoundingSphere::new(translation, self.outer_radius())
    }
}
