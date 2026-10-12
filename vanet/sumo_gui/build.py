"""Build a local SUMO GUI with CARLA coordinate display and clipboard support.

Only four GUI source copies are compiled, reusing the installed SUMO libraries.
The installed SUMO, map projection, simulation and TraCI code are untouched.
"""
import fcntl
import hashlib
import os
from pathlib import Path
import shlex
import shutil
import subprocess

HERE = Path(__file__).resolve().parent


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise RuntimeError('Unsupported SUMO source version: GUI patch location missing or ambiguous')
    return text.replace(old, new, 1)


def ensure_gui():
    home = Path(os.environ.get('SUMO_HOME', Path.home() / 'sumo-1.22.0')).resolve()
    build = Path(os.environ.get('SUMO_BUILD_DIR', home / 'build/cmake-build')).resolve()
    srcbuild = build / 'src'
    linkfile = srcbuild / 'CMakeFiles/sumo-gui.dir/link.txt'
    patches = [
        ('utils/gui/windows', 'utils_gui_windows', 'GUISUMOAbstractView.cpp', [
            ('myApp->getCartesianLabel()->setText(("x:" + toString(pos.x()) + ", y:" + toString(pos.y())).c_str());',
             '''const Position offset = GeoConvHelper::getFinal().getOffset();
    myApp->getCartesianLabel()->setText(("SUMO x:" + toString(pos.x()) + ", y:" + toString(pos.y())
        + " | CARLA x:" + toString(pos.x() - offset.x())
        + ", y:" + toString(offset.y() - pos.y())).c_str());
    myApp->getCartesianLabel()->setTipText("Mouse position in meters. CARLA coordinates use the loaded network's netOffset.");'''),
        ]),
        ('gui', 'gui', 'GUIApplicationWindow.cpp', [
            ('myCartesianFrame->setWidth(textWidth);', 'myCartesianFrame->setWidth(textWidth * 3);'),
        ]),
        ('utils/gui/globjects', 'utils_gui_globjects', 'GUIGlObject.cpp', [
            ('GUIDesigns::buildFXMenuCommand(ret, TL("Copy cursor position to clipboard"), nullptr, ret, MID_COPY_CURSOR_POSITION);',
             '''GUIDesigns::buildFXMenuCommand(ret, "Copy SUMO coordinates", nullptr, ret, MID_COPY_CURSOR_POSITION);
    GUIDesigns::buildFXMenuCommand(ret, "Copy CARLA coordinates", nullptr, ret, MID_LAST + 1);'''),
        ]),
        ('utils/gui/globjects', 'utils_gui_globjects', 'GUIGLObjectPopupMenu.cpp', [
            ('FXMAPFUNC(SEL_COMMAND,  MID_COPY_CURSOR_POSITION,    GUIGLObjectPopupMenu::onCmdCopyCursorPosition),',
             '''FXMAPFUNC(SEL_COMMAND,  MID_COPY_CURSOR_POSITION,    GUIGLObjectPopupMenu::onCmdCopyCursorPosition),
    // Local command after the upstream enum: existing IDs stay unchanged.
    FXMAPFUNC(SEL_COMMAND,  MID_LAST + 1, GUIGLObjectPopupMenu::onCmdCopyCursorPosition),'''),
            ('''GUIGLObjectPopupMenu::onCmdCopyCursorPosition(FXObject*, FXSelector, void*) {
    GUIUserIO::copyToClipboard(*myParent->getApp(), toString(myNetworkPosition));''',
             '''GUIGLObjectPopupMenu::onCmdCopyCursorPosition(FXObject*, FXSelector selector, void*) {
    // Use the saved right-click location, not the pointer over the menu.
    Position pos = myNetworkPosition;
    if (FXSELID(selector) == MID_LAST + 1) {
        const Position offset = GeoConvHelper::getFinal().getOffset();
        pos = Position(pos.x() - offset.x(), offset.y() - pos.y());
    }
    GUIUserIO::copyToClipboard(*myParent->getApp(),
        toString(pos.x(), 6) + ", " + toString(pos.y(), 6));'''),
        ]),
    ]
    work = HERE / 'build'
    work.mkdir(exist_ok=True)
    binary = HERE / 'bin/sumo-gui'
    binary.parent.mkdir(exist_ok=True)
    with (work / 'lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if not linkfile.is_file():
            raise RuntimeError(f'SUMO build artifacts missing: {linkfile}; set SUMO_HOME / SUMO_BUILD_DIR')
        link = shlex.split(linkfile.read_text())
        digest = hashlib.sha256(Path(__file__).read_bytes())
        digest.update(str(build).encode())
        inputs = [linkfile]
        for directory, target, name, _ in patches:
            inputs.extend([home / 'src' / directory / name,
                           srcbuild / directory / f'CMakeFiles/{target}.dir/flags.make'])
        inputs.extend(srcbuild / arg for arg in link if arg.endswith(('.a', '.o', '.so')))
        for path in inputs:
            stat = path.stat()
            digest.update(f'{path}:{stat.st_size}:{stat.st_mtime_ns}'.encode())
        signature = digest.hexdigest()
        stamp = work / 'signature'
        if binary.is_file() and stamp.exists() and stamp.read_text() == signature:
            return binary
        replacements = {}
        for directory, target, name, edits in patches:
            source = home / 'src' / directory / name
            patched = work / name
            text = source.read_text()
            for before, after in edits:
                text = replace_once(text, before, after)
            patched.write_text(text)
            flags = {}
            for line in (srcbuild / directory / f'CMakeFiles/{target}.dir/flags.make').read_text().splitlines():
                if ' = ' in line:
                    key, value = line.split(' = ', 1)
                    flags[key] = shlex.split(value)
            obj = work / (name + '.o')
            command = [link[0], *flags['CXX_DEFINES'], *flags['CXX_INCLUDES'], *flags['CXX_FLAGS'],
                       '-I' + str(source.parent), '-c', str(patched), '-o', str(obj)]
            print('Building project SUMO GUI:', name, flush=True)
            subprocess.run(command, cwd=srcbuild, check=True)
            relative = f'{directory}/lib{target}.a'
            archive = work / ('lib' + target + '.a')
            if relative not in replacements:
                shutil.copy2(srcbuild / relative, archive)
            subprocess.run(['ar', 'r', str(archive), str(obj)], check=True)
            replacements[relative] = str(archive)
        link = [replacements.get(arg, arg) for arg in link]
        temporary = binary.with_name('sumo-gui.new')
        link[link.index('-o') + 1] = str(temporary)
        subprocess.run(link, cwd=srcbuild, check=True)
        os.replace(temporary, binary)
        stamp.write_text(signature)
        return binary


if __name__ == '__main__':
    print(ensure_gui())
