:- module(rosprolog_package,
    [ init_ros_package/1,
      register_ros_package/2,
      register_ros_package/1,
      use_ros_module/2
	]).
/** <module> Loading of rosprolog packages.

@author Daniel Beßler
@license BSD
*/

:- dynamic ros_package_initialized/1.

%% init_ros_package(+PackagePath) is semidet.
%
% Initialize a rosprolog package by consulting the __init__.pl file.
% 
% @param PackagePath  Path towards the package to be initialized (with trailing slash)
%
init_ros_package(PackagePath) :-
  atom_concat(PackagePath, '__init__.pl', InitFile),
  ( exists_file(InitFile)
  -> ( user:consult(InitFile) )
  ;  ( print_message(warning, rosprolog(no_init_pl(InitFile))), fail )
  ).

%% register_ros_package(+Package, ?AbsoluteDirectory) is det.
%
% Find and initialize a rosprolog package.
% Locate the package on the harddisk, add the path
% to the library search path, and consult the __init__.pl
% file that (recursively) initializes the package and its dependencies.
% 
% @param Package            Name of a package
% @param AbsoluteDirectory  Global path to the directory holding __init__.pl.
% 
register_ros_package(Package, _) :-
  ros_package_initialized(Package), !.

register_ros_package(Package, AbsoluteDirectory) :-

print_message(informational, rosprolog(pkg1(Package))),
  ros_package_path(Package, PackagePath),
print_message(informational, rosprolog(pkg2(PackagePath))),
  nonvar(PackagePath),
  atom_concat(PackagePath, '/src/', AbsoluteDirectory),
print_message(informational, rosprolog(pkg3(AbsoluteDirectory))),
  asserta(user:library_directory(AbsoluteDirectory)),
  assert(user:file_search_path(ros, AbsoluteDirectory)),
  assert(ros_package_initialized(Package)),
  ignore(init_ros_package(AbsoluteDirectory)).

%% register_ros_package(+Package) is det.
%
% Same as register_ros_package/2 but ignores the second argument.
% 
% @param Package Name of a rosprolog package
% 
register_ros_package(Package) :-
  register_ros_package(Package, _).

%% use_ros_module(+Package, +FilePath) is semidet.
%
% Load a specific Prolog module declared in some ROS
% package.
% 
% @param Package Name of a rosprolog package
% @param FilePath Relative path to module
%
use_ros_module(Package, FilePath) :-
  register_ros_package(Package, AbsoluteDirectory),
  atom_concat(AbsoluteDirectory, FilePath, AbsoluteFilePath),
  use_module(AbsoluteFilePath).
 