use bevy_math::{
    primitives::{Direction3d, Plane3d},
    Quat, Vec2, Vec3,
};
use wgpu::PrimitiveTopology;

use crate::{
    mesh::{Indices, Mesh, Meshable},
    render_asset::RenderAssetUsages,
};

/// A builder used for creating a [`Mesh`] with a [`Plane3d`] shape.
#[derive(Clone, Copy, Debug)]
pub struct PlaneMeshBuilder {
    /// The [`Plane3d`] shape.
    pub plane: Plane3d,
    /// Half the size of the plane mesh.
    pub half_size: Vec2,
    /// The number of subdivisions in the mesh.
    ///
    /// Examples:
    ///
    /// * 0: The original plane geometry formed by 4 points in the XZ plane.
    /// * 1: The plane is split by 1 line in the middle on both the X axis and Z axes, resulting in a plane with 4 quads / 8 triangles.
    /// * 2: The plane is split by 2 lines on both the X and Z axes, subdividing the plane into 3 equal sections along each axis, resulting in a plane with 9 quads / 18 triangles.
    pub subdivisions: u32,
}

impl Default for PlaneMeshBuilder {
    fn default() -> Self {
        Self {
            plane: Plane3d::default(),
            half_size: Vec2::ONE,
            subdivisions: 0,
        }
    }
}

impl PlaneMeshBuilder {
    /// Creates a new [`PlaneMeshBuilder`] from a given normal and size.
    #[inline]
    pub fn new(normal: Direction3d, size: Vec2) -> Self {
        Self {
            plane: Plane3d { normal },
            half_size: size / 2.0,
            subdivisions: 0,
        }
    }

    /// Creates a new [`PlaneMeshBuilder`] from the given size, with the normal pointing upwards.
    #[inline]
    pub fn from_size(size: Vec2) -> Self {
        Self {
            half_size: size / 2.0,
            ..Default::default()
        }
    }

    /// Sets the normal of the plane, aka the direction the plane is facing.
    #[inline]
    #[doc(alias = "facing")]
    pub fn normal(mut self, normal: Direction3d) -> Self {
        self.plane = Plane3d { normal };
        self
    }

    /// Sets the size of the plane mesh.
    #[inline]
    pub fn size(mut self, width: f32, height: f32) -> Self {
        self.half_size = Vec2::new(width, height) / 2.0;
        self
    }

    /// Sets the number of subdivisions for the plane mesh.
    #[inline]
    pub fn subdivisions(mut self, subdivisions: u32) -> Self {
        self.subdivisions = subdivisions;
        self
    }

    /// Builds a [`Mesh`] based on the configuration in `self`.
    pub fn build(&self) -> Mesh {
        // Split in the X and Z directions if one ever needs asymmetrical subdivision
        let x_vertex_count = self.subdivisions + 2;
        let z_vertex_count = self.subdivisions + 2;
        let num_vertices = (z_vertex_count * x_vertex_count) as usize;
        let num_indices = ((z_vertex_count - 1) * (x_vertex_count - 1) * 6) as usize;

        let mut positions: Vec<[f32; 3]> = Vec::with_capacity(num_vertices);
        let normals: Vec<[f32; 3]> = vec![self.plane.normal.to_array(); num_vertices];
        let mut uvs: Vec<[f32; 2]> = Vec::with_capacity(num_vertices);
        let mut indices: Vec<u32> = Vec::with_capacity(num_indices);

        // The rotation of the plane when a normal pointing towards +Y corresponds to no rotation
        let rotation = Quat::from_rotation_arc(Vec3::Y, *self.plane.normal);

        // The corner where the first vertex is
        let start = rotation * -Vec3::new(self.half_size.x, 0.0, self.half_size.y);

        // How much one step is in the X, Y, and Z directions
        let step = rotation * Vec3::new(2.0 * self.half_size.x, 0.0, 2.0 * self.half_size.y)
            / (self.subdivisions + 1) as f32;

        // Add vertices.
        for z in 0..z_vertex_count {
            for x in 0..x_vertex_count {
                let (x, z) = (x as f32, z as f32);
                let tx = x / (x_vertex_count - 1) as f32;
                let tz = z / (z_vertex_count - 1) as f32;
                positions.push([
                    start.x + x * step.x,
                    start.y + (x + z) * step.y,
                    start.z + z * step.z,
                ]);
                uvs.push([tx, tz]);
            }
        }

        // Add indices.
        for y in 0..z_vertex_count - 1 {
            for x in 0..x_vertex_count - 1 {
                let quad = y * x_vertex_count + x;
                indices.extend_from_slice(&[
                    quad + x_vertex_count + 1,
                    quad + 1,
                    quad + x_vertex_count,
                    quad,
                    quad + x_vertex_count,
                    quad + 1,
                ]);
            }
        }

        Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::default(),
        )
        .with_inserted_indices(Indices::U32(indices))
        .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, positions)
        .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normals)
        .with_inserted_attribute(Mesh::ATTRIBUTE_UV_0, uvs)
    }
}

impl Meshable for Plane3d {
    type Output = PlaneMeshBuilder;

    fn mesh(&self) -> Self::Output {
        PlaneMeshBuilder {
            plane: *self,
            ..Default::default()
        }
    }
}

impl From<Plane3d> for Mesh {
    fn from(plane: Plane3d) -> Self {
        plane.mesh().build()
    }
}

impl From<PlaneMeshBuilder> for Mesh {
    fn from(plane: PlaneMeshBuilder) -> Self {
        plane.build()
    }
}
