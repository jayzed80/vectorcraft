//! Hostile numbers in attributes (1e308, NaN-looking, negative, zero, huge dash arrays, deep
//! nesting): import, export and render must finish without panicking or hanging.

use std::time::{Duration, Instant};
use vectorcraft_svg::{ExportOptions, export, import};
use vectorcraft_testkit::raster::render_artboard;

const NS: &str = "xmlns=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http://www.w3.org/1999/xlink\"";

fn wrap(attrs: &str, body: &str) -> String {
    format!("<svg {NS} {attrs}>{body}</svg>")
}

fn cases() -> Vec<String> {
    let nest = 300;
    let deep = format!("{}<rect width=\"5\" height=\"5\"/>{}", "<g>".repeat(nest), "</g>".repeat(nest));
    let many_dashes = (0..2000).map(|i| format!("{}", 1 + i % 7)).collect::<Vec<_>>().join(",");
    vec![
        wrap(
            "width=\"100\" height=\"100\"",
            "<path d=\"M0 0 L50 50\" stroke=\"red\" stroke-width=\"1e200\" stroke-linejoin=\"round\" fill=\"none\"/>",
        ),
        wrap("width=\"100\" height=\"100\"", "<path d=\"M0 0 L50 50 L0 50Z\" stroke=\"red\" stroke-width=\"1e308\" stroke-linecap=\"round\"/>"),
        wrap("width=\"100\" height=\"100\"", "<path d=\"M0 0 L50 50\" stroke=\"red\" stroke-width=\"-5\" stroke-miterlimit=\"0\"/>"),
        wrap(
            "width=\"100\" height=\"100\"",
            "<path d=\"M0 0 L50 50\" stroke=\"red\" stroke-dasharray=\"1e-300,1e-300\" stroke-dashoffset=\"1e308\"/>",
        ),
        wrap("width=\"100\" height=\"100\"", &format!("<path d=\"M0 0 L5000 5\" stroke=\"red\" stroke-dasharray=\"{many_dashes}\"/>")),
        wrap("width=\"100\" height=\"100\"", "<path d=\"M0 0 L5000000 5\" stroke=\"red\" stroke-dasharray=\"0.0001\"/>"),
        wrap("width=\"1e300\" height=\"1e300\"", "<rect width=\"10\" height=\"10\"/>"),
        wrap("width=\"0\" height=\"0\" viewBox=\"0 0 0 0\"", "<rect width=\"10\" height=\"10\"/>"),
        wrap("width=\"100\" height=\"100\" viewBox=\"0 0 1e-300 1e-300\"", "<rect width=\"10\" height=\"10\"/>"),
        wrap("width=\"100\" height=\"100\" viewBox=\"-1e308 -1e308 1e308 1e308\"", "<rect width=\"10\" height=\"10\"/>"),
        wrap("width=\"100\" height=\"100\"", "<rect width=\"10\" height=\"10\" transform=\"scale(1e200) rotate(1e300) skewX(89.9999999)\"/>"),
        wrap("width=\"100\" height=\"100\"", "<rect width=\"10\" height=\"10\" transform=\"matrix(0 0 0 0 0 0)\"/>"),
        wrap("width=\"100\" height=\"100\"", "<rect width=\"10\" height=\"10\" rx=\"1e308\" ry=\"-3\"/>"),
        wrap("width=\"100\" height=\"100\"", "<circle r=\"1e308\" cx=\"1e308\" cy=\"-1e308\"/>"),
        wrap("width=\"100\" height=\"100\"", "<ellipse rx=\"0\" ry=\"1e-320\"/>"),
        wrap("width=\"100\" height=\"100\"", "<polygon points=\"1e308,1e308 -1e308,1e308 0,-1e308 abc 5\"/>"),
        wrap("width=\"100\" height=\"100\"", "<path d=\"M0 0 A1e308 1e-308 1e308 1 1 100 100 A0 0 0 0 0 5 5 C1e308 0 0 1e308 1 1\"/>"),
        wrap(
            "width=\"100\" height=\"100\"",
            "<defs><linearGradient id=\"g\" x1=\"1e308\" x2=\"-1e308\" gradientTransform=\"scale(0)\"><stop offset=\"1e300\" stop-color=\"red\"/><stop offset=\"-1e300\"/></linearGradient></defs><rect width=\"10\" height=\"10\" fill=\"url(#g)\"/>",
        ),
        wrap(
            "width=\"100\" height=\"100\"",
            "<defs><radialGradient id=\"g\" r=\"-1\" fr=\"1e308\" cx=\"1e308\"><stop offset=\"0\" stop-color=\"red\"/></radialGradient></defs><rect width=\"10\" height=\"10\" fill=\"url(#g)\"/>",
        ),
        wrap(
            "width=\"100\" height=\"100\"",
            "<defs><filter id=\"f\"><feGaussianBlur stdDeviation=\"1e308\"/><feOffset dx=\"1e308\"/></filter></defs><rect width=\"10\" height=\"10\" filter=\"url(#f)\"/>",
        ),
        wrap(
            "width=\"100\" height=\"100\"",
            "<defs><filter id=\"f\" x=\"-1e308\" width=\"1e308\"><feMorphology radius=\"1e9\"/><feConvolveMatrix order=\"1e9\"/></filter></defs><rect width=\"10\" height=\"10\" filter=\"url(#f)\"/>",
        ),
        wrap(
            "width=\"100\" height=\"100\"",
            "<defs><rect id=\"a\" width=\"5\" height=\"5\"/></defs><use xlink:href=\"#a\" x=\"1e308\" y=\"nan\" width=\"-3\"/>",
        ),
        wrap("width=\"100\" height=\"100\"", "<g id=\"a\"><use xlink:href=\"#a\"/></g><use xlink:href=\"#a\"/>"),
        wrap("width=\"100\" height=\"100\"", "<text font-size=\"1e308\" x=\"1e308\" y=\"5\" letter-spacing=\"1e308\" textLength=\"-1\">hello</text>"),
        wrap("width=\"100\" height=\"100\"", "<text font-size=\"0\" stroke=\"red\" stroke-width=\"1e200\">hello</text>"),
        wrap("width=\"100\" height=\"100\"", "<rect width=\"10\" height=\"10\" opacity=\"1e308\" fill-opacity=\"-5\" stroke-opacity=\"nan\"/>"),
        wrap("width=\"100\" height=\"100\"", "<rect width=\"10\" height=\"10\" style=\"stroke-width:1e999;fill:rgb(1e999,-1e999,nan)\"/>"),
        wrap("width=\"100\" height=\"100\"", &deep),
    ]
}

#[test]
fn hostile_numbers_never_panic_or_hang() {
    for (i, svg) in cases().iter().enumerate() {
        let start = Instant::now();
        // An import error is acceptable; a panic or a hang is not.
        if let Ok((doc, _)) = vectorcraft_svg::import_with_report(svg) {
            let _ = export(&doc, &ExportOptions::default());
            if let Some(ab) = doc.artboards.first() {
                // Rendering a giant artboard is the renderer's business; only draw sane sizes.
                if ab.rect.width() <= 4096.0 && ab.rect.height() <= 4096.0 {
                    let _ = render_artboard(&doc);
                }
            }
        }
        let _ = import(svg);
        assert!(start.elapsed() < Duration::from_secs(20), "case {i} took {:?}: {svg}", start.elapsed());
    }
}
