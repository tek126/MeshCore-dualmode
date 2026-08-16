# Injects the git short SHA into FIRMWARE_VERSION for car_node builds, so the
# on-device `ver` command shows e.g. "v1.17.1+carnode-2c319911" — making it
# obvious which build is actually flashed. Referenced via extra_scripts in the
# *_carnode envs. FIRMWARE_VERSION is guarded by #ifndef in MyMesh.h, so this
# -D wins.
#
# Project sources (examples/simple_repeater/*) are compiled with `projenv`, so
# the define must be appended there — appending only to `env` does not reach them.
import subprocess

Import("env")


def git_describe():
    try:
        rev = subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"],
            stderr=subprocess.DEVNULL).decode().strip()
    except Exception:
        return "nogit"
    dirty = subprocess.call(["git", "diff", "--quiet"],
                            stderr=subprocess.DEVNULL) != 0
    return rev + ("-dirty" if dirty else "")


version = "v1.17.1+carnode-%s" % git_describe()

try:
    macro = env.StringifyMacro(version)      # portable quoting (PlatformIO)
except AttributeError:
    macro = '\\"%s\\"' % version             # fallback: manual escaped quotes

env.Append(CPPDEFINES=[("FIRMWARE_VERSION", macro)])
try:
    Import("projenv")
    projenv.Append(CPPDEFINES=[("FIRMWARE_VERSION", macro)])
except Exception as e:
    print("car_node_version: projenv not available (%s)" % e)

print("car_node firmware version: %s  (macro=%s)" % (version, macro))
