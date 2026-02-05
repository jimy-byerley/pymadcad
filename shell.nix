# nix-python-shell with stuff required by pymadcad to run inside a nix environment

with import <nixpkgs> { };

let
  py = python310Packages;
  libPath = lib.makeLibraryPath [
      stdenv.cc.cc.lib  # libstdc++ required by glm
      libglvnd  # vendor neutral gl lib, needed by madcad.render
      zlib  # compression lib that is required by numpy, etc  
    ];

  
in pkgs.mkShell rec {
  name = "pyzone";
  venvDir = "./.venv";
  buildInputs = [
    # A Python interpreter including the 'venv' module is required to bootstrap
    # the environment.
    py.python

    # This executes some shell code to initialize a venv in $venvDir before
    # dropping into the shell
    py.venvShellHook

    # Those are dependencies that we would like to use from nixpkgs, which will
    # add them to PYTHONPATH and thus make them accessible from within the venv.
    py.pyqt5
   
    # utils
    git
  ];

  # Run this command, only after creating the virtual environment
  postVenvCreation = ''
    unset SOURCE_DATE_EPOCH

    # devtools
    pip install pytest
    
    # install pymadcad in editable mode
    pip install -e .
    # if one just want use pymadcad:
    #   pip install pymadcad  
    '';

  # Now we can execute any commands within the virtual environment.
  # This is optional and can be left out to run pip manually.
  
  postShellHook = ''
    # allow pip to install wheels
    export LD_LIBRARY_PATH=${libPath}
    unset SOURCE_DATE_EPOCH
  '';

  QT_QPA_PLATFORM_PLUGIN_PATH="${qt5.qtbase.bin}/lib/qt-${qt5.qtbase.version}/plugins";
}