"""Build the actual Veins radio extension without changing installed dependencies."""
import fcntl
import hashlib
import json
import os
import shutil
import subprocess
from pathlib import Path
from vanet.project_paths import (
    OMNET_PROJECT_DIRECTORY, OMNET_INSTALL_DIRECTORY, VEINS_INSTALL_DIRECTORY,
    OMNET_COMPILED_OUTPUT_DIRECTORY, OMNET_WIRELESS_LIBRARY_FILE, OMNET_BUILD_LOCK_FILE,
    OMNET_INCLUDE_DIRECTORY, OMNET_LIBRARY_DIRECTORY, VEINS_SOURCE_DIRECTORY,
    VEINS_WIRELESS_LIBRARY_FILE, OMNET_COBRIDGE_INTERFACE_DIRECTORY, OMNET_WIRELESS_APPLICATIONS_DIRECTORY,
)


def build_network(force=False):
    output = OMNET_COMPILED_OUTPUT_DIRECTORY
    output.mkdir(parents=True, exist_ok=True)
    OMNET_BUILD_LOCK_FILE.parent.mkdir(parents=True, exist_ok=True)
    sources = sorted(OMNET_COBRIDGE_INTERFACE_DIRECTORY.glob('*.cc')) + sorted(OMNET_WIRELESS_APPLICATIONS_DIRECTORY.glob('*.cc'))
    headers = sorted(OMNET_COBRIDGE_INTERFACE_DIRECTORY.glob('*.h')) + sorted(OMNET_WIRELESS_APPLICATIONS_DIRECTORY.glob('*.h'))
    compiler = shutil.which('g++')
    if not compiler:
        raise RuntimeError('g++ is required to build the Veins network extension')
    include = OMNET_INCLUDE_DIRECTORY
    omnet_lib = OMNET_LIBRARY_DIRECTORY
    veins_src = VEINS_SOURCE_DIRECTORY
    for required in (include / 'omnetpp.h', VEINS_WIRELESS_LIBRARY_FILE):
        if not required.exists():
            raise FileNotFoundError(f'Missing simulation dependency: {required}')
    temporary_library = OMNET_WIRELESS_LIBRARY_FILE.with_name(OMNET_WIRELESS_LIBRARY_FILE.name + '.building')
    command = [compiler, '-std=c++17', '-O2', '-fPIC', '-shared', '-pthread',
               '-I' + str(include), '-I' + str(veins_src), *map(str, sources),
               '-L' + str(omnet_lib), '-L' + str(veins_src), '-lveins', '-loppsim', '-loppcommon',
               '-Wl,-rpath,' + str(omnet_lib), '-Wl,-rpath,' + str(veins_src),
               '-o', str(temporary_library)]
    signature = hashlib.sha256(json.dumps(command).encode() + b''.join(p.read_bytes() for p in sources + headers)).hexdigest()
    signature_file = output / 'build_signature.txt'
    with OMNET_BUILD_LOCK_FILE.open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if not force and OMNET_WIRELESS_LIBRARY_FILE.exists() and signature_file.exists() and signature_file.read_text() == signature:
            return OMNET_WIRELESS_LIBRARY_FILE
        print('[OMNeT++] Compiling real Veins IEEE 802.11p extension', flush=True)
        result = subprocess.run(command, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        (output / 'build.log').write_text(result.stdout)
        if result.returncode:
            print(result.stdout, flush=True)
            raise RuntimeError(f'OMNeT++ extension build failed; see {output / "build.log"}')
        temporary_library.replace(OMNET_WIRELESS_LIBRARY_FILE)
        signature_file.write_text(signature)
    return OMNET_WIRELESS_LIBRARY_FILE


if __name__ == '__main__':
    print(build_network())
