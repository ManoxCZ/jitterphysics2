/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using Jitter2.LinearMath;

namespace Jitter2.Dynamics.Buoyancy;

/// <summary>
/// Defines a water surface that can exert buoyancy forces on rigid bodies.
/// Implement this interface to provide custom water shapes such as flat planes,
/// bounded volumes, or animated wave surfaces.
/// </summary>
public interface IWaterSurface
{
    /// <summary>
    /// Returns the water height (Y coordinate) at the given world-space XZ position.
    /// Points with a world Y below this height are considered submerged.
    /// </summary>
    Real GetHeight(Real x, Real z);

    /// <summary>
    /// Returns the upward surface normal at the given world-space XZ position.
    /// For flat surfaces this is always <see cref="JVector.UnitY"/>.
    /// </summary>
    JVector GetNormal(Real x, Real z) => JVector.UnitY;

    /// <summary>
    /// Density of the fluid in kg/m³. Pure water is approximately 1000.
    /// </summary>
    Real FluidDensity { get; }

    /// <summary>
    /// Linear drag coefficient applied to the velocity of submerged bodies.
    /// Acts as a multiplier on the velocity-opposing force per unit of submerged volume.
    /// </summary>
    Real LinearDrag { get; }

    /// <summary>
    /// Angular drag coefficient applied to the angular velocity of submerged bodies.
    /// Acts as a multiplier on the angular-velocity-opposing torque per unit of submerged volume.
    /// </summary>
    Real AngularDrag { get; }

    /// <summary>
    /// Returns true if the given world-space position is within the region covered by this water surface.
    /// Use this to restrict buoyancy to a bounded area (e.g., a lake or pool).
    /// The default implementation covers the infinite XZ plane.
    /// </summary>
    bool Contains(in JVector worldPosition) => true;
}
