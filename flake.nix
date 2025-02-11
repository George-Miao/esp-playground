{
  description = "ESP dev shell";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    esp32 = {
      url = "github:leighleighleigh/esp-rs-nix";
      inputs.nixpkgs.follows = "nixpkgs";
      inputs.flake-utils.follows = "flake-utils";
    };
  };

  outputs = {
    nixpkgs,
    flake-utils,
    esp32,
    ...
  }:
    flake-utils.lib.eachDefaultSystem (
      system: let
        pkgs = import nixpkgs {
          inherit system;
        };
      in {
        devShells.default = pkgs.mkShell {
          inputsFrom = [
            esp32.devShells.${system}.default
          ];
          packages = with pkgs; [
            clang
            esp-generate
            cargo-feature
          ];
        };
      }
    );
}
