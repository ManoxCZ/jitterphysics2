/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using Jitter2.Dynamics.Buoyancy;
using JitterDemo.Renderer;
using JitterDemo.Renderer.OpenGL;

namespace JitterDemo;

/// <summary>
/// Renders <see cref="IWaterSurface"/> instances as semi-transparent blue triangle grids
/// via <see cref="DebugRenderer"/>. Call <see cref="Draw"/> from
/// <see cref="IDrawUpdate.DrawUpdate"/> each frame.
/// </summary>
public static class WaterSurfaceDebugDraw
{
    /// <summary>
    /// Draws <paramref name="surface"/> as a blue triangle grid.
    /// </summary>
    /// <param name="surface">Water surface to visualise.</param>
    /// <param name="debug">The debug renderer to push triangles into.</param>
    /// <param name="cellSize">Size of each grid cell in metres.</param>
    public static void Draw(IWaterSurface surface, DebugRenderer debug, float cellSize = 2.0f)
    {
        switch (surface)
        {
            case WaterVolume vol:
                DrawVolume(vol, debug, cellSize);
                break;
            case WaterPlane plane:
                DrawPlane(plane, debug, cellSize);
                break;
        }
    }

    // Finite 40 × 40 m grid centred on the world origin — enough for any pool-sized scene.
    private static void DrawPlane(WaterPlane plane, DebugRenderer debug, float cellSize)
    {
        const float halfExtent = 20.0f;
        DrawGrid(debug,
            -halfExtent, -halfExtent,
             halfExtent,  halfExtent,
            (float)plane.WaterLevel, cellSize);
    }

    // Grid covering the exact XZ footprint of the volume's top face.
    private static void DrawVolume(WaterVolume vol, DebugRenderer debug, float cellSize)
    {
        DrawGrid(debug,
            (float)(vol.Center.X - vol.HalfExtents.X),
            (float)(vol.Center.Z - vol.HalfExtents.Z),
            (float)(vol.Center.X + vol.HalfExtents.X),
            (float)(vol.Center.Z + vol.HalfExtents.Z),
            (float)vol.WaterLevel, cellSize);
    }

    private static void DrawGrid(DebugRenderer debug,
        float minX, float minZ, float maxX, float maxZ,
        float y, float cellSize)
    {
        int nx = Math.Max(1, (int)MathF.Ceiling((maxX - minX) / cellSize));
        int nz = Math.Max(1, (int)MathF.Ceiling((maxZ - minZ) / cellSize));

        float stepX = (maxX - minX) / nx;
        float stepZ = (maxZ - minZ) / nz;

        for (int ix = 0; ix < nx; ix++)
        {
            for (int iz = 0; iz < nz; iz++)
            {
                float x0 = minX + ix * stepX;
                float x1 = x0 + stepX;
                float z0 = minZ + iz * stepZ;
                float z1 = z0 + stepZ;

                var v00 = new Vector3(x0, y, z0);
                var v10 = new Vector3(x1, y, z0);
                var v01 = new Vector3(x0, y, z1);
                var v11 = new Vector3(x1, y, z1);

                debug.PushTriangle(DebugRenderer.Color.Blue, v00, v10, v11);
                debug.PushTriangle(DebugRenderer.Color.Blue, v00, v11, v01);
            }
        }
    }
}
