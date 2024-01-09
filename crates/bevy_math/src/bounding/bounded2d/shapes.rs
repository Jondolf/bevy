use glam::Vec2;

use crate::primitives::{Circle, Rectangle};

use super::{rotate_vec2, Aabb2d, Bounded2d, BoundingCircle};

impl Bounded2d for Circle {
    fn aabb_2d(&self, translation: Vec2, _rotation: f32) -> Aabb2d {
        Aabb2d {
            min: translation - Vec2::splat(self.radius),
            max: translation + Vec2::splat(self.radius),
        }
    }

    fn bounding_circle(&self, translation: Vec2) -> BoundingCircle {
        BoundingCircle {
            center: translation,
            circle: *self,
        }
    }
}

impl Bounded2d for Rectangle {
    fn aabb_2d(&self, translation: Vec2, rotation: f32) -> Aabb2d {
        let half_size = Vec2::new(self.half_width, self.half_height);
        let half_extents = rotate_vec2(half_size, rotation);

        Aabb2d {
            min: translation - half_extents,
            max: translation + half_extents,
        }
    }

    fn bounding_circle(&self, translation: Vec2) -> BoundingCircle {
        let half_size = Vec2::new(self.half_width, self.half_height);
        BoundingCircle {
            center: translation,
            circle: Circle {
                radius: half_size.length(),
            },
        }
    }
}
