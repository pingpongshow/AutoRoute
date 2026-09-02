//! AutoRoute core: board model, routing grid, and the negotiated-congestion
//! grid router. No I/O and no UI — see `autoroute-dsn` for Specctra interop.

pub mod board;
pub mod drc;
pub mod geometry;
pub mod grid;
pub mod refine;
pub mod router;

pub use board::{Board, Net, Pad, PadShape, Rules};
pub use router::{route_board, Progress, RouteResult, RouteSeg, RoutedNet, Router, RouterConfig};
