/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using Jitter2.LinearMath;

namespace Jitter2.Dynamics.Buoyancy;

/// <summary>
/// An infinite flat water surface at a constant world-space Y level.
/// </summary>
public class WaterPlane : IWaterSurface
{
    /// <summary>
    /// The Y coordinate of the water surface in world space.
    /// </summary>
    public Real WaterLevel { get; set; }

    /// <inheritdoc/>
    public Real FluidDensity { get; set; } = (Real)1000.0;

    /// <inheritdoc/>
    public Real LinearDrag { get; set; } = (Real)0.9;

    /// <inheritdoc/>
    public Real AngularDrag { get; set; } = (Real)0.95;

    /// <summary>
    /// Creates a new <see cref="WaterPlane"/> at the specified Y level.
    /// </summary>
    /// <param name="waterLevel">World-space Y coordinate of the water surface. Default is 0.</param>
    public WaterPlane(Real waterLevel = (Real)0.0)
    {
        WaterLevel = waterLevel;
    }

    /// <inheritdoc/>
    public Real GetHeight(Real x, Real z) => WaterLevel;
}
