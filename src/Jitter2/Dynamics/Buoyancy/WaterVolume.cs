/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using Jitter2.LinearMath;

namespace Jitter2.Dynamics.Buoyancy;

/// <summary>
/// A flat water surface bounded by an axis-aligned box region.
/// Only rigid bodies whose shapes overlap with this volume receive buoyancy forces.
/// </summary>
public class WaterVolume : IWaterSurface
{
    /// <summary>
    /// The center of the water volume in world space.
    /// The water surface sits at <c>Center.Y + HalfExtents.Y</c>.
    /// </summary>
    public JVector Center { get; set; }

    /// <summary>
    /// The half-extents of the axis-aligned bounding box for this water volume.
    /// </summary>
    public JVector HalfExtents { get; set; }

    /// <inheritdoc/>
    public Real FluidDensity { get; set; } = (Real)1000.0;

    /// <inheritdoc/>
    public Real LinearDrag { get; set; } = (Real)0.9;

    /// <inheritdoc/>
    public Real AngularDrag { get; set; } = (Real)0.95;

    /// <summary>
    /// Creates a new <see cref="WaterVolume"/>.
    /// </summary>
    /// <param name="center">Center of the water volume in world space.</param>
    /// <param name="halfExtents">Half-extents of the bounding box.</param>
    public WaterVolume(JVector center, JVector halfExtents)
    {
        Center = center;
        HalfExtents = halfExtents;
    }

    /// <summary>
    /// The Y coordinate of the water surface (top face of the volume).
    /// </summary>
    public Real WaterLevel => Center.Y + HalfExtents.Y;

    /// <inheritdoc/>
    public Real GetHeight(Real x, Real z) => WaterLevel;

    /// <summary>
    /// Returns true if the point is within the XZ footprint of this water volume
    /// and below the water surface.
    /// </summary>
    public bool Contains(in JVector worldPosition)
    {
        return worldPosition.X >= Center.X - HalfExtents.X &&
               worldPosition.X <= Center.X + HalfExtents.X &&
               worldPosition.Z >= Center.Z - HalfExtents.Z &&
               worldPosition.Z <= Center.Z + HalfExtents.Z &&
               worldPosition.Y <= WaterLevel;
    }
}
