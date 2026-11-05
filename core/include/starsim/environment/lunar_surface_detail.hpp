#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include "starsim/config/moon_surface_config.hpp"
#include "starsim/environment/cube_face_cell.hpp"
#include "starsim/environment/lunar_terrain.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::environment {

// One generated small crater. Profile (simple crater, rim scaled with depth): parabolic bowl from the floor at -depth
// + rim to the rim crest at r = R, rim_height ((r/R)^-3 - 1/27) / (1 - 1/27) outside, zero from r = 3R
// (docs/subsystems/moon-surface.md, Small craters).
struct Crater {
    math::Vec3 centre_unit;
    double diameter_m = 0.0;
    double depth_m = 0.0;       // rim crest to floor
    double rim_height_m = 0.0;  // rim crest above the surrounding surface
    bool fresh = false;
};

// Relief of a crater at distance_m from its centre (m, added to the surface).
double crater_profile_m(const Crater& crater, double distance_m);

// Deterministic procedural relief of the Moon below the LOLA grid: small craters and self-affine regolith roughness,
// a pure function of the body-fixed (MOON_ME) direction (integer hash, no state, no library random numbers). Rocks are
// a separate layer (LunarRocks) on top of this relief.
//
// Zero mean: LOLA heights are area means, so every crater octave enters as its deviation from the population's mean
// relief (CraterOctave::mean_relief_m) and the roughness is zero-mean value noise - the detail does not shift LOLA.
//
// Faces: the roughness lattices belong to one cube face each (continued beyond its edges); within face_blend_band of
// a face edge the field is the face_blend of the 2-3 face fields (weights / sqrt(sum w^2): same variance), so the
// relief at any L is continuous across edges and corners and depends on the direction only.
//
// Band limits: a query with min_wavelength_m = L > 0 returns the mean of the full-detail relief over the footprint
// p + a e_s + b e_t, |a|, |b| <= L/2 (the face lattice's (s, t) parallelogram; across an edge the face_blend-weighted
// set of the faces' parallelograms): craters of diameter >= L/8 enter with their exact area mean (polygon integral),
// smaller ones fade out over L/16 - L/8 (their mean deviation is ~0); roughness octaves of lattice spacing >= L/8 with
// their exact mean (separable; over another face's sheared parallelogram by a composite midpoint rule), finer ones
// fade out over L/16 - L/8. L = 0 is the full detail.
//
// Bands: craters up to 5 x band_start_m (at the crater centre) and roughness octaves up to band_start_m, the LOLA
// resolution at the point blended by the tile weight - what LOLA resolves is not generated again.
class LunarSurfaceDetail {
public:
    // Throws std::invalid_argument for a null terrain or an unknown size-frequency form.
    LunarSurfaceDetail(config::MoonSurfaceConfig config, std::shared_ptr<const LunarTerrain> lola);

    // Relief added to LOLA at `unit` with every feature shorter than min_wavelength_m averaged out (0: full detail).
    double relief_m(const math::Vec3& unit, double min_wavelength_m) const;
    double crater_relief_m(const math::Vec3& unit, double min_wavelength_m) const;
    double roughness_relief_m(const math::Vec3& unit, double min_wavelength_m) const;
    double band_start_m(const math::Vec3& unit) const;  // LOLA resolution blended by tile_weight
    double finest_wavelength_m() const;                  // ~0.01 m (the craters' smallest diameter)
    const config::MoonSurfaceConfig& config() const { return config_; }
    const LunarTerrain& lola() const { return *lola_; }
    double sphere_radius_m() const { return sphere_radius_m_; }

    // Crater population. Craters of one diameter octave live in the face cells of one level (centres inside the cell).
    struct CraterOctave {
        double min_diameter_m = 0.0, max_diameter_m = 0.0;
        int level = 0;                 // face-cell level (cells >= 3.34 x max diameter: the reach 1.5 D <= 0.45 cell)
        double density_per_m2 = 0.0;   // N(min) - N(max)
        double mean_relief_m = 0.0;    // expected mean relief of the octave's craters per unit area (m)
        double cumulative_min_per_m2 = 0.0, cumulative_max_per_m2 = 0.0;  // N(min), N(max) (precomputed draws)
    };
    const std::vector<CraterOctave>& crater_octaves() const { return crater_octaves_; }
    // Generated craters with centres in the cell (empty for a level no octave uses).
    std::vector<Crater> craters_in_cell(const FaceCell& cell) const;
    // Generated craters with diameter in [min_diameter_m, max_diameter_m) and centre within radius_m (chord) of unit.
    std::vector<Crater> craters_within(const math::Vec3& unit, double radius_m, double min_diameter_m,
                                       double max_diameter_m) const;

    // Roughness: value-noise octaves (quintic fade) on per-octave face lattices with a prime number of cells per face
    // edge (no lattice line shared between octaves), amplitude = scale x (local spacing / baseline)^H.
    struct RoughnessOctave {
        std::int64_t cells_per_edge = 0;
        double spacing_m = 0.0;  // nominal (face centre)
        double relative_amplitude = 0.0;  // sqrt 3 (spacing / baseline)^H: x scale x (local metric scale)^H
    };
    const std::vector<RoughnessOctave>& roughness_octaves() const { return roughness_octaves_; }
    // Amplitude scale (m), derived at construction: median plane-detrended RMS height of the generated roughness in
    // baseline windows (25 x 25 samples, Guo et al. 2021's statistic) = roughness.rms_height_at_baseline_m.
    double roughness_scale_m() const { return roughness_scale_m_; }

private:
    struct Query;
    // Per-thread memo of generated crater cells (direct-mapped by the cell hash): nearby queries share most cells, and
    // generating a cell (count, positions, completion) dominated a point query. A hit returns the same draws, so the
    // relief is bit for bit the one generated in place.
    struct CraterCellMemo {
        static constexpr int slots = 2048;     // [E] ~1 MB per thread
        static constexpr int max_draws = 6;    // cells with more draws (rare) are generated in place
        struct Slot {  // the key and positions first (read for every cell), the craters after (read on a near draw)
            std::uint64_t instance = 0;  // 0: empty
            int octave = -1, face = -1, count = 0;
            std::int64_t i = 0, j = 0;
            std::array<double, max_draws> s01{}, t01{};
            std::array<bool, max_draws> complete{};
            std::array<Crater, max_draws> crater;
        };
        std::vector<Slot> table = std::vector<Slot>(slots);
    };
    const CraterCellMemo::Slot* memo_cell(int octave, const FaceCell& cell, std::uint64_t cell_hash) const;
    Query make_query(const math::Vec3& unit, bool for_craters = true) const;  // false: face blend only
    double craters_at(const Query& q, double min_wavelength_m) const;
    double roughness_at(const Query& q, double min_wavelength_m, double scale_m) const;
    void craters_in_cell_of(int octave, const FaceCell& cell, std::vector<Crater>& out) const;
    int crater_count(int octave, const FaceCell& cell, std::uint64_t cell_hash) const;
    // Completes draw m of the cell (position s01, t01 already drawn); false where LOLA resolves the crater.
    bool complete_crater(int octave, const FaceCell& cell, std::uint64_t cell_hash, int m, double s01, double t01,
                         Crater& out) const;
    double crater_mean_field_m(int octave, double band_start_m) const;
    // Every octave's crater_mean_field_m at band_start_m, memoised per thread for the last band start (the same values;
    // the field is a 32-step sum that otherwise dominates a point query). Valid until this thread's next call.
    const double* crater_mean_fields(double band_start_m) const;
    // Every roughness octave's top weight smooth01(log2(band_start / spacing)), memoised per thread likewise.
    const double* roughness_top_weights(double band_start_m) const;
    double roughness_node(int octave, int face, std::int64_t j, std::int64_t k) const;
    double derive_roughness_scale() const;

    config::MoonSurfaceConfig config_;
    std::shared_ptr<const LunarTerrain> lola_;
    double sphere_radius_m_ = 0.0;
    double finest_band_start_m_ = 0.0;  // lower bound of band_start_m over the Moon
    double rim_to_depth_ = 0.0;         // kappa = rim height / depth (rim scaled with depth)
    std::vector<CraterOctave> crater_octaves_;
    std::vector<RoughnessOctave> roughness_octaves_;
    double roughness_scale_m_ = 0.0;
    std::uint64_t instance_id_ = 0;  // unique per instance (keys the per-thread mean-field memo)
};

}  // namespace starsim::environment
