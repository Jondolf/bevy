use crate::{
    mesh::{Indices, Mesh},
    render_asset::RenderAssetPersistencePolicy,
};

use super::{Facing, MeshFacingExtension, Meshable};
use bevy_math::primitives::Ellipse;
use wgpu::PrimitiveTopology;

/// A builder used for creating a [`Mesh`] with an [`Ellipse`] shape.
#[derive(Clone, Copy, Debug)]
pub struct EllipseMesh {
    /// The [`Ellipse`] shape.
    pub ellipse: Ellipse,
    /// The number of vertices used for the ellipse mesh.
    /// The default is `32`.
    #[doc(alias = "vertices")]
    pub resolution: usize,
    /// The XYZ direction that the mesh is facing.
    /// The default is [`Facing::Z`].
    pub facing: Facing,
}

impl Default for EllipseMesh {
    fn default() -> Self {
        Self {
            ellipse: Ellipse::default(),
            resolution: 32,
            facing: Facing::Z,
        }
    }
}

impl MeshFacingExtension for EllipseMesh {
    #[inline]
    fn facing(mut self, facing: Facing) -> Self {
        self.facing = facing;
        self
    }
}

impl EllipseMesh {
    /// Creates a new [`EllipseMesh`] from a given half width and half height and a vertex count.
    #[inline]
    pub const fn new(half_width: f32, half_height: f32, resolution: usize) -> Self {
        Self {
            ellipse: Ellipse {
                half_width,
                half_height,
            },
            resolution,
            facing: Facing::Z,
        }
    }

    /// Sets the number of vertices used for the ellipse mesh.
    #[inline]
    #[doc(alias = "vertices")]
    pub const fn resolution(mut self, resolution: usize) -> Self {
        self.resolution = resolution;
        self
    }

    /// Builds a [`Mesh`] based on the configuration in `self`.
    pub fn build(&self) -> Mesh {
        let mut indices = Vec::with_capacity((self.resolution - 2) * 3);
        let mut positions = Vec::with_capacity(self.resolution);
        let mut normals = Vec::with_capacity(self.resolution);
        let mut uvs = Vec::with_capacity(self.resolution);

        self.build_mesh_data(
            [0.0; 3],
            &mut indices,
            &mut positions,
            &mut normals,
            &mut uvs,
        );

        Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetPersistencePolicy::Keep,
        )
        .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, positions)
        .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normals)
        .with_inserted_attribute(Mesh::ATTRIBUTE_UV_0, uvs)
        .with_indices(Some(Indices::U32(indices)))
    }

    pub(super) fn build_mesh_data(
        &self,
        translation: [f32; 3],
        indices: &mut Vec<u32>,
        positions: &mut Vec<[f32; 3]>,
        normals: &mut Vec<[f32; 3]>,
        uvs: &mut Vec<[f32; 2]>,
    ) {
        let sides = self.resolution;
        let [trans_x, trans_y, trans_z] = translation;

        let index_offset = positions.len() as u32;
        let facing_coords = self.facing.to_array();
        let normal_sign = self.facing.signum() as f32;
        let step = normal_sign * std::f32::consts::TAU / sides as f32;

        for i in 0..sides {
            let theta = std::f32::consts::FRAC_PI_2 + i as f32 * step;
            let (sin, cos) = theta.sin_cos();
            let x = cos * self.ellipse.half_width;
            let y = sin * self.ellipse.half_height;

            let position = match self.facing {
                Facing::X | Facing::NegX => [trans_x, trans_y + y, trans_z - x],
                Facing::Y | Facing::NegY => [trans_x + x, trans_y, trans_z - y],
                Facing::Z | Facing::NegZ => [trans_x + x, trans_y + y, trans_z],
            };

            positions.push(position);
            normals.push(facing_coords);
            uvs.push([0.5 * (cos + 1.0), 1.0 - 0.5 * (sin + 1.0)]);
        }

        for i in 1..(sides as u32 - 1) {
            indices.extend_from_slice(&[index_offset, index_offset + i, index_offset + i + 1]);
        }
    }
}

impl Meshable for Ellipse {
    type Output = EllipseMesh;

    fn mesh(&self) -> Self::Output {
        EllipseMesh {
            ellipse: *self,
            ..Default::default()
        }
    }
}

impl From<Ellipse> for Mesh {
    fn from(ellipse: Ellipse) -> Self {
        ellipse.mesh().build()
    }
}

impl From<EllipseMesh> for Mesh {
    fn from(ellipse: EllipseMesh) -> Self {
        ellipse.build()
    }
}
