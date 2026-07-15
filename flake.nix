{
  description = "Nix flake for state-obervation";

  inputs.mc-rtc-nix.url = "github:mc-rtc/nixpkgs";

  outputs =
    inputs:
    inputs.mc-rtc-nix.lib.mkFlakoboros inputs (
      { lib, ... }:
      {
        overrideAttrs.state-observation =
          { drv-prev, pkgs-final, ... }:
          {
            src = lib.cleanSource ./.;
            nativeBuildInputs = [ pkgs-final.jrl-cmakemodules ] ++ drv-prev.nativeBuildInputs;
          };
      }
    );
}
