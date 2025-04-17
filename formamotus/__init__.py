import os
import subprocess
import sys

BPY_AVAILABLE = False
try:
    import bpy
    BPY_AVAILABLE = True

    class FormaMotusPanel(bpy.types.Panel):
        bl_label = "FormaMotus"
        bl_idname = "VIEW3D_PT_forma_motus"
        bl_space_type = 'VIEW_3D'
        bl_region_type = 'UI'
        bl_category = "FormaMotus"

        def draw(self, context):
            layout = self.layout
            scene = context.scene

            # File paths
            box = layout.box()
            box.label(text="File Paths")
            box.prop(scene, "formamotus_urdf_filepath")
            box.prop(scene, "formamotus_render_filepath")

            # Visualization options
            box = layout.box()
            box.label(text="Visualization Options")
            box.prop(scene, "formamotus_use_mesh", text="Use Mesh Visualization")
            box.label(text="Cylinder Size (mm)")
            box.prop(scene, "formamotus_cylinder_radius", slider=True)
            box.prop(scene, "formamotus_cylinder_height", slider=True)
            box.prop(scene, "formamotus_connector_cylinder_radius", slider=True)

            # Joint color settings
            box = layout.box()
            box.label(text="Joint Colors")
            box.prop(scene, "formamotus_revolute_color")
            box.prop(scene, "formamotus_prismatic_color")
            box.prop(scene, "formamotus_continuous_color")
            box.prop(scene, "formamotus_default_color")

            # Operator buttons
            layout.operator(robot_visualizer.RobotVisualizerOperator.bl_idname, text="Visualize Robot")
            layout.operator(robot_visualizer.RobotRenderOperator.bl_idname, text="Render Image")

            # Robot model info
            robot_model = robot_visualizer.get_robot_model()
            layout.label(text=f"Robot Model: {robot_model if robot_model else 'Not loaded'}")

            # Joint angle sliders
            if robot_model:
                box = layout.box()
                box.label(text="Joint Angles")
                for joint_name in robot_model.joint_names:
                    joint = robot_model.__dict__.get(joint_name)
                    if joint and joint.type != 'fixed':
                        prop_name = f"formamotus_joint_angle_{joint_name.replace(' ', '_').replace('/', '_')}"
                        if hasattr(scene, prop_name):
                            box.prop(scene, prop_name, slider=True)
            else:
                layout.label(text="No robot model loaded.", icon='ERROR')

except ImportError:
    pass

version = '1.1.2'
repository = 'https://github.com/iory/formamotus'

bl_info = {
    "name": "FormaMotus",
    "description": "Visualize robot kinematics structures in Blender.",
    "author": "Iori Yanokura",
    "version": (1, 1, 2),
    "blender": (3, 0, 0),
    "location": "View3D > Sidebar > FormaMotus",
    "warning": "",
    "tracker_url": "https://github.com/iory/formamotus/issues",
    "category": "3D View",
}

requirements = {
    "numpy": "numpy",
    "skrobot": "scikit-robot",
}

optional_requirements = {}
extra_requirements = {}

installation_finished_message = "All FormaMotus requirements have been installed.\nPlease restart Blender to activate the FormaMotus add-on!"

def install_requirement(package_name, upgrade_pip=False, lib=None, ensure_pip=True):
    if lib is None and BPY_AVAILABLE:
        lib = bpy.utils.user_resource("SCRIPTS", path="modules")
    if ensure_pip:
        try:
            subprocess.check_call([sys.executable, "-m", "ensurepip", "--user"])
        except subprocess.CalledProcessError:
            print("WARNING: We couldn't do ensurepip, we try to continue anyways")
            pass
    if upgrade_pip:
        print("  Upgrading pip...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "--upgrade", "pip"])
    print("  Installing package", package_name)
    if lib is None:
        subprocess.check_call([sys.executable, "-m", "pip", "install", "--upgrade", package_name])
    else:
        subprocess.check_call([sys.executable, "-m", "pip", "install", "--upgrade", f"--target={lib!s}", package_name])

def check_requirements(optional=False, extra=False, force=False, upgrade_pip=False, lib=None, install=True):
    import importlib
    print("Checking requirements:")
    try:
        subprocess.check_call([sys.executable, "-m", "ensurepip", "--user"])
    except subprocess.CalledProcessError:
        print("WARNING: We couldn't do ensurepip, we try to continue anyways")
        pass
    reqs = [requirements]
    if optional:
        reqs += [optional_requirements]
    if extra:
        reqs += [extra_requirements]
    if upgrade_pip:
        print("  Upgrading pip...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "--upgrade", "pip"])
    for r in reqs:
        for import_name, req_name in r.items():
            print("  Checking", import_name)
            try:
                try:
                    if importlib.util.find_spec(import_name) is None:
                        if install:
                            install_requirement(req_name, upgrade_pip=False, lib=lib, ensure_pip=False)
                        else:
                            raise ImportError("Uninstalled requirement: " + req_name)
                except AttributeError:
                    loader = importlib.find_loader(import_name)
                    if not issubclass(type(loader), importlib.machinery.SourceFileLoader):
                        if install:
                            install_requirement(req_name, upgrade_pip=False, lib=lib, ensure_pip=False)
                        else:
                            raise ImportError("Uninstalled requirement: " + req_name)
            except subprocess.CalledProcessError as e:
                if import_name in list(optional_requirements.keys()) + list(extra_requirements.keys()):
                    print(f"Couldn't install optional requirement {import_name} ({req_name})")
                else:
                    raise e
    importlib.invalidate_caches()
    import skrobot  # NOQA

def register():
    """This function registers all modules to Blender."""
    try:
        addon_dir = os.path.dirname(__file__)
        sys.path.append(os.path.join(addon_dir, "libs"))

        from . import robot_visualizer

        # Recursively import all submodules
        def import_submodules(package, recursive=True, verbose=False):
            import importlib
            import pkgutil
            import sys

            modules = sys.modules
            if isinstance(package, str):
                package = importlib.import_module(package)

            results = {}
            for _loader, name, is_pkg in pkgutil.walk_packages(package.__path__):
                full_name = package.__name__ + '.' + name
                if full_name in modules.keys():
                    if verbose:
                        print("RELOAD: ", full_name)
                    results[full_name] = importlib.reload(modules[full_name])
                else:
                    if verbose:
                        print("IMPORT: ", full_name)
                    results[full_name] = importlib.import_module(full_name)
                if recursive and is_pkg:
                    results.update(import_submodules(full_name))
            return results

        print("Importing FormaMotus")
        robot_visualizer.register()
        bpy.utils.register_class(FormaMotusPanel)

    except ImportError:
        def draw(self, context):
            self.layout.label(text=installation_finished_message)
        bpy.context.window_manager.popup_menu(draw, title="FormaMotus: Please restart Blender")

def unregister():
    """This function unregisters all modules in Blender."""
    print("\n" + "-" * 100)
    print("Unregistering FormaMotus...")
    from . import robot_visualizer

    bpy.utils.unregister_class(FormaMotusPanel)
    robot_visualizer.unregister()

if "blender" not in sys.executable.lower() and not BPY_AVAILABLE:
    try:
        from importlib.metadata import PackageNotFoundError
        from importlib.metadata import version
    except ImportError:
        try:
            from importlib_metadata import PackageNotFoundError
            from importlib_metadata import version
        except ImportError:
            from pkg_resources import DistributionNotFound as PackageNotFoundError
            from pkg_resources import get_distribution
            def version(package_name):
                return get_distribution("formamotus").version
    try:
        __version__ = version("formamotus")
    except PackageNotFoundError:
        __version__ = ".".join([str(x) for x in bl_info["version"]])
    del version, PackageNotFoundError

if BPY_AVAILABLE:
    try:
        from . import robot_visualizer
    except ImportError:
        check_requirements(optional=True, upgrade_pip=True, extra=False, install=True)
        print('\033[92m' + '\033[1m' + "FormaMotus: " + installation_finished_message + '\033[0m')
else:
    from . import robot_visualizer
