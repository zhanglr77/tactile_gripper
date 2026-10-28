"""
magnet_field.py

Finite cylindrical permanent magnet magnetic-field model.

Coordinate convention
---------------------
- Geometry unit: mm
- Magnetic field unit: mT
- Each magnet is centered at magnet_position
- magnet_n defines the magnetization direction
- Magnet local +Z axis is aligned with magnet_n

The magnet geometry and magnetic parameters are imported from
magnetic_model.py.

Model
-----
A uniformly magnetized finite cylindrical magnet is represented by
two magnetic surface-charge disks:

    top surface:    +M
    bottom surface: -M

The magnetic field is calculated by numerical surface integration.

This model is intended mainly for points outside the magnet.
"""

import numpy as np

MAGNET_RADIUS = 1.5
MAGNET_HEIGHT = 2.0
MAGNET_REMANENCE = 850.0


# ============================================================
# Numerical integration configuration
# ============================================================

# Number of radial subdivisions
DEFAULT_NR = 30

# Number of angular subdivisions
DEFAULT_NPHI = 120


# ============================================================
# Basic vector utilities
# ============================================================

def normalize(v):
    """
    Normalize a 3D vector.

    Parameters
    ----------
    v : array-like, shape (3,)

    Returns
    -------
    ndarray, shape (3,)
        Unit vector.
    """

    v = np.asarray(v, dtype=float)

    norm = np.linalg.norm(v)

    if norm < 1e-12:
        raise ValueError("Cannot normalize zero vector.")

    return v / norm


# ============================================================
# Magnet coordinate system
# ============================================================

def build_local_frame(magnet_n):
    """
    Construct the local coordinate frame of a cylindrical magnet.

    Local +Z:
        aligned with magnetization direction magnet_n.

    Local X/Y:
        automatically constructed to form a right-handed frame.

    Parameters
    ----------
    magnet_n : array-like, shape (3,)
        Magnetization direction expressed in global coordinates.

    Returns
    -------
    R : ndarray, shape (3, 3)

        Rotation matrix:

            vector_global = R @ vector_local

        Columns of R are:

            [ex_global, ey_global, ez_global]
    """

    ez = normalize(magnet_n)

    # Select a reference direction that is not almost parallel to ez
    if abs(ez[2]) < 0.9:
        reference = np.array([0.0, 0.0, 1.0])
    else:
        reference = np.array([1.0, 0.0, 0.0])

    # Construct local X
    ex = np.cross(reference, ez)
    ex = normalize(ex)

    # Construct local Y
    ey = np.cross(ez, ex)
    ey = normalize(ey)

    R = np.column_stack([
        ex,
        ey,
        ez
    ])

    return R


# ============================================================
# Coordinate transformations
# ============================================================

def global_to_magnet_local(
    point_global,
    magnet_position,
    magnet_n
):
    """
    Convert an observation point from global coordinates
    to magnet-local coordinates.

    Parameters
    ----------
    point_global : array-like, shape (3,)
        Observation point in global XYZ [mm].

    magnet_position : array-like, shape (3,)
        Magnet center in global XYZ [mm].

    magnet_n : array-like, shape (3,)
        Magnetization direction in global XYZ.

    Returns
    -------
    point_local : ndarray, shape (3,)
        Observation point relative to magnet center,
        expressed in magnet-local coordinates [mm].
    """

    point_global = np.asarray(point_global, dtype=float)
    magnet_position = np.asarray(magnet_position, dtype=float)

    R = build_local_frame(magnet_n)

    relative_global = (
        point_global
        - magnet_position
    )

    point_local = (
        R.T
        @ relative_global
    )

    return point_local


def local_to_global_vector(
    vector_local,
    magnet_n
):
    """
    Transform a vector from magnet-local coordinates
    to global coordinates.

    Parameters
    ----------
    vector_local : array-like, shape (3,)
        Vector expressed in magnet-local coordinates.

    magnet_n : array-like, shape (3,)
        Magnetization direction in global coordinates.

    Returns
    -------
    vector_global : ndarray, shape (3,)
    """

    vector_local = np.asarray(
        vector_local,
        dtype=float
    )

    R = build_local_frame(magnet_n)

    return R @ vector_local


# ============================================================
# Analytical axial magnetic field
# ============================================================

def axial_field_exact(
    z,
    radius=MAGNET_RADIUS,
    height=MAGNET_HEIGHT,
    Br=MAGNET_REMANENCE,
):
    """
    Exact axial magnetic field of a uniformly magnetized
    finite cylindrical magnet.

    The magnet is centered at z = 0 and magnetized along +Z.

    Parameters
    ----------
    z : float
        Observation position along the magnet axis,
        measured from magnet center [mm].

    radius : float
        Magnet radius [mm].

    height : float
        Magnet height [mm].

    Br : float
        Effective remanent flux density [mT].

    Returns
    -------
    Bz : float
        Axial magnetic field [mT].

    Formula
    -------
    Bz(z) = Br/2 *
        [
            (z + h/2) / sqrt(R^2 + (z + h/2)^2)
            -
            (z - h/2) / sqrt(R^2 + (z - h/2)^2)
        ]
    """

    half_height = height / 2.0

    term_top = (
        z + half_height
    ) / np.sqrt(
        radius**2
        + (z + half_height)**2
    )

    term_bottom = (
        z - half_height
    ) / np.sqrt(
        radius**2
        + (z - half_height)**2
    )

    Bz = (
        0.5
        * Br
        * (term_top - term_bottom)
    )

    return Bz


# ============================================================
# Finite cylinder field - local coordinate system
# ============================================================

def finite_cylinder_field_local(
    point_local,
    radius=MAGNET_RADIUS,
    height=MAGNET_HEIGHT,
    Br=MAGNET_REMANENCE,
    nr=DEFAULT_NR,
    nphi=DEFAULT_NPHI,
):
    """
    Calculate magnetic field produced by a finite cylindrical
    permanent magnet.

    The magnet is:

        centered at (0, 0, 0)

        local magnetization:
            +Z

        cylindrical axis:
            local Z

    Parameters
    ----------
    point_local : array-like, shape (3,)
        Observation point [x, y, z] in magnet-local frame [mm].

    radius : float
        Magnet radius [mm].

    height : float
        Magnet height [mm].

    Br : float
        Effective remanent flux density [mT].

    nr : int
        Number of radial integration subdivisions.

    nphi : int
        Number of angular integration subdivisions.

    Returns
    -------
    B_local : ndarray, shape (3,)
        Magnetic field:

            [Bx, By, Bz]

        in magnet-local coordinates [mT].

    Theory
    ------
    Uniform axial magnetization can be represented by magnetic
    surface charge.

    Top circular face:

        sigma_m = +M

    Bottom circular face:

        sigma_m = -M

    For observation points outside the magnet:

        B(r) =
            Br / (4*pi)
            integral[
                sign *
                (r-r') / |r-r'|^3
                dA
            ]

    where Br = mu0*M.
    """

    point_local = np.asarray(
        point_local,
        dtype=float
    )

    if nr <= 0:
        raise ValueError(
            "nr must be greater than zero."
        )

    if nphi <= 0:
        raise ValueError(
            "nphi must be greater than zero."
        )

    # --------------------------------------------------------
    # Convert geometry:
    # mm -> m
    # --------------------------------------------------------

    p = point_local * 1e-3

    radius_m = (
        radius * 1e-3
    )

    height_m = (
        height * 1e-3
    )

    # Magnetic field:
    # mT -> T
    Br_T = (
        Br * 1e-3
    )


    # --------------------------------------------------------
    # Integration increments
    # --------------------------------------------------------

    dr = (
        radius_m / nr
    )

    dphi = (
        2.0
        * np.pi
        / nphi
    )


    # --------------------------------------------------------
    # Constant coefficient
    # --------------------------------------------------------

    coefficient = (
        Br_T
        / (4.0 * np.pi)
    )


    # --------------------------------------------------------
    # Total field
    # --------------------------------------------------------

    B = np.zeros(
        3,
        dtype=float
    )


    # --------------------------------------------------------
    # Surface locations
    #
    # top:
    #   z = +height/2
    #   surface charge +
    #
    # bottom:
    #   z = -height/2
    #   surface charge -
    # --------------------------------------------------------

    surfaces = [
        (
            +1.0,
            +height_m / 2.0
        ),
        (
            -1.0,
            -height_m / 2.0
        ),
    ]


    # --------------------------------------------------------
    # Numerical integration
    # --------------------------------------------------------

    for (
        surface_sign,
        z_surface
    ) in surfaces:

        for ir in range(nr):

            # Midpoint quadrature
            rho = (
                (ir + 0.5)
                * dr
            )

            for iphi in range(nphi):

                phi = (
                    (iphi + 0.5)
                    * dphi
                )

                # Source point on circular surface
                source = np.array([
                    rho * np.cos(phi),
                    rho * np.sin(phi),
                    z_surface,
                ])


                # Vector:
                #
                # source -> observation
                #
                r_vec = (
                    p - source
                )

                distance = (
                    np.linalg.norm(r_vec)
                )


                # Avoid numerical singularity
                if distance < 1e-12:
                    continue


                # Surface area element
                #
                # dA = rho d(rho) d(phi)
                #
                dA = (
                    rho
                    * dr
                    * dphi
                )


                # Field contribution
                dB = (
                    coefficient
                    * surface_sign
                    * r_vec
                    / distance**3
                    * dA
                )

                B += dB


    # --------------------------------------------------------
    # Tesla -> mT
    # --------------------------------------------------------

    B_mT = (
        B * 1e3
    )

    return B_mT


# ============================================================
# Finite cylinder field - global coordinate system
# ============================================================

def finite_cylinder_field(
    point_global,
    magnet_position,
    magnet_n,
    radius=MAGNET_RADIUS,
    height=MAGNET_HEIGHT,
    Br=MAGNET_REMANENCE,
    nr=DEFAULT_NR,
    nphi=DEFAULT_NPHI,
):
    """
    Calculate magnetic field of one finite cylindrical magnet
    at an arbitrary global point.

    Parameters
    ----------
    point_global : array-like, shape (3,)
        Observation point in global XYZ [mm].

    magnet_position : array-like, shape (3,)
        Magnet center in global XYZ [mm].

    magnet_n : array-like, shape (3,)
        Magnetization direction in global XYZ.

    radius : float
        Magnet radius [mm].

    height : float
        Magnet height [mm].

    Br : float
        Effective remanence [mT].

    nr, nphi : int
        Numerical integration resolution.

    Returns
    -------
    B_global : ndarray, shape (3,)
        Magnetic field [Bx, By, Bz] in global coordinates [mT].
    """

    # Observation point:
    # global -> magnet local
    point_local = (
        global_to_magnet_local(
            point_global=point_global,
            magnet_position=magnet_position,
            magnet_n=magnet_n,
        )
    )


    # Magnetic field in magnet-local coordinates
    B_local = (
        finite_cylinder_field_local(
            point_local=point_local,
            radius=radius,
            height=height,
            Br=Br,
            nr=nr,
            nphi=nphi,
        )
    )


    # Magnetic field:
    # local -> global
    B_global = (
        local_to_global_vector(
            B_local,
            magnet_n
        )
    )

    return B_global


# ============================================================
# Multiple magnets
# ============================================================

def total_field(
    point_global,
    magnets,
    radius=MAGNET_RADIUS,
    height=MAGNET_HEIGHT,
    Br=MAGNET_REMANENCE,
    nr=DEFAULT_NR,
    nphi=DEFAULT_NPHI,
):
    """
    Calculate total magnetic field from multiple magnets.

    Magnetic fields are linearly superimposed.

    Parameters
    ----------
    point_global : array-like, shape (3,)
        Observation point [mm].

    magnets : dict

        Expected format:

        {
            "mag1": {
                "position": np.array([x, y, z]),
                "n": np.array([nx, ny, nz])
            },

            ...
        }

    Returns
    -------
    B_total : ndarray, shape (3,)
        Total magnetic field in global coordinates [mT].
    """

    B_total = np.zeros(
        3,
        dtype=float
    )

    for magnet in magnets.values():

        B_magnet = (
            finite_cylinder_field(
                point_global=point_global,
                magnet_position=magnet["position"],
                magnet_n=magnet["n"],
                radius=radius,
                height=height,
                Br=Br,
                nr=nr,
                nphi=nphi,
            )
        )

        B_total += B_magnet

    return B_total


# ============================================================
# Individual magnet contributions
# ============================================================

def field_contributions(
    point_global,
    magnets,
    radius=MAGNET_RADIUS,
    height=MAGNET_HEIGHT,
    Br=MAGNET_REMANENCE,
    nr=DEFAULT_NR,
    nphi=DEFAULT_NPHI,
):
    """
    Calculate magnetic field contribution from each magnet.

    Returns
    -------
    contributions : dict

        Example:

        {
            "mag1": np.array([Bx, By, Bz]),
            "mag2": np.array([Bx, By, Bz]),
            ...
        }
    """

    contributions = {}

    for (
        magnet_name,
        magnet
    ) in magnets.items():

        contributions[magnet_name] = (
            finite_cylinder_field(
                point_global=point_global,
                magnet_position=magnet["position"],
                magnet_n=magnet["n"],
                radius=radius,
                height=height,
                Br=Br,
                nr=nr,
                nphi=nphi,
            )
        )

    return contributions


# ============================================================
# Validation
# ============================================================

def validate_axis_model(
    z_values=None,
    nr=DEFAULT_NR,
    nphi=DEFAULT_NPHI,
):
    """
    Compare numerical finite-cylinder integration with the
    exact analytical axial solution.

    Parameters
    ----------
    z_values : iterable
        Distances from magnet center [mm].

    Returns
    -------
    results : list of dict
    """

    if z_values is None:
        z_values = [
            1.5,
            2.0,
            3.0,
            5.0,
            10.0,
        ]

    results = []

    for z in z_values:

        point = np.array([
            0.0,
            0.0,
            z
        ])

        B_numeric = (
            finite_cylinder_field_local(
                point_local=point,
                nr=nr,
                nphi=nphi,
            )
        )

        B_exact = (
            axial_field_exact(z)
        )

        error = (
            B_numeric[2]
            - B_exact
        )

        if abs(B_exact) > 1e-12:

            relative_error = (
                error
                / B_exact
                * 100.0
            )

        else:

            relative_error = np.nan


        results.append({
            "z": z,
            "B_numeric": B_numeric,
            "B_exact": B_exact,
            "error": error,
            "relative_error_percent": relative_error,
        })

    return results


# ============================================================
# Main test
# ============================================================

if __name__ == "__main__":

    print("=" * 70)
    print("FINITE CYLINDER MAGNET FIELD MODEL")
    print("=" * 70)

    print()
    print("Magnet parameters imported from magnetic_model.py")
    print()

    print(
        f"Radius     : "
        f"{MAGNET_RADIUS:.4f} mm"
    )

    print(
        f"Height     : "
        f"{MAGNET_HEIGHT:.4f} mm"
    )

    print(
        f"Remanence : "
        f"{MAGNET_REMANENCE:.4f} mT"
    )


    # ========================================================
    # Surface axial field from analytical solution
    # ========================================================

    surface_z = (
        MAGNET_HEIGHT / 2.0
    )

    B_surface_exact = (
        axial_field_exact(
            surface_z
        )
    )

    print()
    print("-" * 70)

    print(
        "Analytical axial field "
        "at magnet surface:"
    )

    print(
        f"z  = {surface_z:.4f} mm"
    )

    print(
        f"Bz = {B_surface_exact:.4f} mT"
    )


    # ========================================================
    # Compare numerical and analytical models
    # ========================================================

    print()
    print("-" * 70)
    print("AXIS MODEL VALIDATION")
    print("-" * 70)

    results = (
        validate_axis_model()
    )

    print()

    print(
        f"{'z [mm]':>10}"
        f"{'Numeric Bz [mT]':>20}"
        f"{'Exact Bz [mT]':>20}"
        f"{'Error [%]':>15}"
    )

    print("-" * 65)

    for result in results:

        print(
            f"{result['z']:>10.3f}"
            f"{result['B_numeric'][2]:>20.6f}"
            f"{result['B_exact']:>20.6f}"
            f"{result['relative_error_percent']:>15.4f}"
        )