{
  description = "Dev flake for C++ Embedded development with Platform IO";
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-25.05";
  };
  outputs =
    inputs:
    let
      supportedSystems = [
        "x86_64-linux"
        "aarch64-linux"
      ];

      forEachSupportedSystem =
        f:
        inputs.nixpkgs.lib.genAttrs supportedSystems (
          system:
          f {
            pkgs = import inputs.nixpkgs { inherit system; };
            inherit system;
          }
        );
    in
    {
      formatter.x86_64-linux = inputs.nixpkgs.legacyPackages.x86_64-linux.nixfmt-tree;

      devShells = forEachSupportedSystem (
        { pkgs, system }:
        {
          default = pkgs.mkShell {
            buildInputs = [
              pkgs.python3
            ];
          };
          shellHook = ''ASTRA Socket dev'';
        }
      );
    };
}
