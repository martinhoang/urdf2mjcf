## TODO Bugs/Features
[x] Raise error on failed mesh conversion
[x] Apply special operations, e.g. "inject_attrs", "replace_attrs", etc. with w.r.t to the <tag> scope of the parent tag
[x] Save json file with relative package path if relative paths were given at original cli / config json file
[ ] Passsing additional arguments along side json config file
[ ] Processing multiple meshes (with their colors) inside a DAE file
[ ] Re-arrange post-processing procedures for max functionalities and avoid potential bugs
[ ] Unit tests this whole package
[x] Activate tool usage in cli arguments for more mesh conversion and processing functions
[x] Install some of the tools as extension of the "urdf2mjcf" tool, perhaps through a flag or use dot "." notation or something
[ ] Use CoACD for convex decomposition to achieve high-resolution collision hulls
[x] Automatically use tool to reduce mesh complexity, i.e. number of faces to be between 1-200000
