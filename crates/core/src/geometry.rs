//! Minimal 2D geometry helpers. All coordinates are f64 micrometers.

pub type P = [f64; 2];

#[inline]
pub fn sub(a: P, b: P) -> P {
    [a[0] - b[0], a[1] - b[1]]
}

#[inline]
pub fn dot(a: P, b: P) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

#[inline]
pub fn dist(a: P, b: P) -> f64 {
    let d = sub(a, b);
    (d[0] * d[0] + d[1] * d[1]).sqrt()
}

/// Distance from point `p` to segment `a`-`b`.
pub fn seg_dist(p: P, a: P, b: P) -> f64 {
    let ab = sub(b, a);
    let len2 = dot(ab, ab);
    if len2 < 1e-12 {
        return dist(p, a);
    }
    let t = (dot(sub(p, a), ab) / len2).clamp(0.0, 1.0);
    dist(p, [a[0] + ab[0] * t, a[1] + ab[1] * t])
}

/// Even-odd rule point-in-polygon test.
pub fn point_in_poly(p: P, poly: &[P]) -> bool {
    let n = poly.len();
    if n < 3 {
        return false;
    }
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let (a, b) = (poly[i], poly[j]);
        if (a[1] > p[1]) != (b[1] > p[1]) {
            let x = a[0] + (p[1] - a[1]) / (b[1] - a[1]) * (b[0] - a[0]);
            if p[0] < x {
                inside = !inside;
            }
        }
        j = i;
    }
    inside
}

/// Minimum distance from `p` to the polygon outline (edges).
pub fn poly_edge_dist(p: P, poly: &[P]) -> f64 {
    let n = poly.len();
    let mut best = f64::INFINITY;
    for i in 0..n {
        let d = seg_dist(p, poly[i], poly[(i + 1) % n]);
        if d < best {
            best = d;
        }
    }
    best
}

/// Signed distance to a polygon: negative inside, positive outside.
pub fn poly_signed_dist(p: P, poly: &[P]) -> f64 {
    let d = poly_edge_dist(p, poly);
    if point_in_poly(p, poly) {
        -d
    } else {
        d
    }
}

/// Rotate a point around the origin by `deg` degrees counter-clockwise.
pub fn rotate(p: P, deg: f64) -> P {
    let r = deg.to_radians();
    let (s, c) = r.sin_cos();
    [p[0] * c - p[1] * s, p[0] * s + p[1] * c]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_point_in_poly() {
        let sq = [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]];
        assert!(point_in_poly([5.0, 5.0], &sq));
        assert!(!point_in_poly([15.0, 5.0], &sq));
    }

    #[test]
    fn test_signed_dist() {
        let sq = [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]];
        assert!((poly_signed_dist([5.0, 5.0], &sq) + 5.0).abs() < 1e-9);
        assert!((poly_signed_dist([12.0, 5.0], &sq) - 2.0).abs() < 1e-9);
    }

    #[test]
    fn test_seg_dist() {
        assert!((seg_dist([5.0, 3.0], [0.0, 0.0], [10.0, 0.0]) - 3.0).abs() < 1e-9);
        assert!((seg_dist([-4.0, 3.0], [0.0, 0.0], [10.0, 0.0]) - 5.0).abs() < 1e-9);
    }
}
