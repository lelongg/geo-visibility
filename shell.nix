let
  rust-overlay = builtins.fetchTarball {
    url =
      "https://github.com/oxalica/rust-overlay/tarball/339de634e6f19e420e81d5a6acaa39364a726999";
    sha256 = "sha256:180lfb32ldkc2rdw0r2f6mfbxav5f7n328lm7s1h7zw8440inn65";
  };
  pkgs = import <nixpkgs> { overlays = [ (import (rust-overlay)) ]; };
in with pkgs;
let
  rust =
    rust-bin.stable.latest.default.override { extensions = [ "rust-src" ]; };
  rustPlatform = makeRustPlatform {
    rustc = rust;
    cargo = rust;
  };
in mkShell {
  buildInputs = [ clang rust openssl pkgconfig ];
  LIBCLANG_PATH = "${llvmPackages.libclang}/lib";
}
