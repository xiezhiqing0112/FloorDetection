import math

# WGS-84 geodetic constants
a = 6378137  # WGS-84 Earth semimajor axis (m)
b = 6356752.3142  # WGS-84 Earth semiminor axis (m)
f = (a - b) / a  # Ellipsoid Flatness
e_sq = f * (2 - f)  # Square of Eccentricity


# Converts WGS-84 Geodetic point (lat, lon, h) to the
# Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z).
def GeodeticToEcef(lat, lon, h):
    # Convert to math.radians in notation consistent with the paper:
    # lambda i sa reserved word
    lamb = math.radians(lat)
    phi = math.radians(lon)
    s = math.sin(lamb)
    N = a / math.sqrt(1 - e_sq * s * s)

    sin_lambda = math.sin(lamb)
    cos_lambda = math.cos(lamb)
    cos_phi = math.cos(phi)
    sin_phi = math.sin(phi)

    x = (h + N) * cos_lambda * cos_phi
    y = (h + N) * cos_lambda * sin_phi
    z = (h + (1 - e_sq) * N) * sin_lambda
    return (x, y, z)


# Converts the Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z) to
# East-North-Up coordinates in a Local Tangent Plane that is centered at the
# (WGS-84) Geodetic point (lat0, lon0, h0).
def EcefToEnu(x, y, z, lat0, lon0, h0):
    # Convert to math.radians in notation consistent with the paper:
    lamb = math.radians(lat0)
    phi = math.radians(lon0)
    s = math.sin(lamb)
    N = a / math.sqrt(1 - e_sq * s * s)

    sin_lambda = math.sin(lamb)
    cos_lambda = math.cos(lamb)
    cos_phi = math.cos(phi)
    sin_phi = math.sin(phi)

    x0 = (h0 + N) * cos_lambda * cos_phi
    y0 = (h0 + N) * cos_lambda * sin_phi
    z0 = (h0 + (1 - e_sq) * N) * sin_lambda

    xd = x - x0
    yd = y - y0
    zd = z - z0

    # This is the matrix multiplication
    xEast = -sin_phi * xd + cos_phi * yd
    yNorth = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd
    zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd
    return (xEast, yNorth, zUp)


# Converts the geodetic WGS-84 coordinated (lat, lon, h) to
# East-North-Up coordinates in a Local Tangent Plane that is centered at the
# (WGS-84) Geodetic point (lat0, lon0, h0).
def GeodeticToEnu(lat, lon, h, lat0, lon0, h0):
    x, y, z = GeodeticToEcef(lat, lon, h)
    xEast, yNorth, zUp = EcefToEnu(x, y, z, lat0, lon0, h0)
    return (xEast, yNorth, zUp)